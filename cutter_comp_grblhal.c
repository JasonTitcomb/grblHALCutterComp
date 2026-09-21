/*
 * SHIM BETWEEN GRBLHAL AND THE CUTTER COMPENSATION CORE
 * Overall, this file serves as a bridge between grblHAL and the cutter compensation core, enabling them to work together seamlessly while keeping their internal implementations decoupled.

 * code is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef CUTTER_COMP_GRBLHAL_H
#define CUTTER_COMP_GRBLHAL_H
#include "grbl/config.h"

#ifdef __cplusplus
extern "C"
{
#endif

#if CUTTER_COMP_ENABLE

#include <stdio.h>

#include "cutter_comp.h"

#include "grbl/hal.h"
#include "grbl/state_machine.h"
#include "grbl/protocol.h"
#include "grbl/motion_control.h"

    static bool cc_mc_is_active(void);

    static plan_line_data_t cc_mc_active_plan_data = {0};
    static bool cc_mc_have_plan_data = false;
    static bool cc_mc_active = false;
    static volatile bool cc_mc_pause_after_next_motion = false;
    static volatile bool cc_mc_output_paused = false;
    static float cc_mc_pending_dwell = 0.0f;
    static float cc_mc_input_pos[N_AXIS] = {0};
    static comp_side cc_mc_saved_comp_side = CC_COMP_OFF;
    static comp_mode cc_mc_saved_comp_mode = CC_CM_NONE;
    static bool cc_mc_saved_state_valid = false;
    static gc_ccomp_t cc;
    static plane_t plane;

    // cutter_comp_grblhal.h

    static on_program_paused_ptr on_program_paused;
    static on_report_options_ptr on_report_options;
    static on_control_signals_changed_ptr cc_prev_on_control_signals_changed = NULL;
    static on_state_change_ptr on_state_change;
    static on_pre_gcode_execute_ptr on_pre_gcode_execute;
    static on_modal_state_action_ptr on_modal_state_action;
    static mc_line_ptr core_mc_line;
    static mc_arc_ptr core_mc_arc;
    static bool cc_control_hook_attached = false;

    static void cc_on_control_signals_changed(control_signals_t signals)
    {
        if (signals.single_block && sys.flags.single_block && state_get() == STATE_CYCLE) {
            if (cc_mc_is_active())
                cc_mc_pause_after_next_motion = true; // mid-compensation: defer to a safe point instead of stopping now
            else
                system_set_exec_state_flag(EXEC_FEED_HOLD); // no compensation in progress, stop now instead of waiting for the next line
        }

        if (cc_prev_on_control_signals_changed)
            cc_prev_on_control_signals_changed(signals);
    }

    static inline void cc_mc_install_control_hook(void)
    {
        if (cc_control_hook_attached)
            return;

        cc_control_hook_attached = true;
        cc_prev_on_control_signals_changed = grbl.on_control_signals_changed;
        grbl.on_control_signals_changed = cc_on_control_signals_changed;
    }

    static void cc_on_state_change(sys_state_t state)
    {
        if (on_state_change)
            on_state_change(state);

        if (state == STATE_IDLE && cc_mc_output_paused) {
            cc_mc_output_paused = false;
            cc_api_drain_output();
        }
    }

    static status_code_t set_options (setting_id_t id, uint_fast16_t int_value)
    {
        settings.flags.cc_chamfer_corner = int_value & 0x1;
    #if CUTTER_COMP_ENABLE == 2
        settings.flags.cc_lookahead_enable = !!(int_value & 0x2);
    #endif

        return Status_OK;
    }

    static uint32_t get_options (setting_id_t id)
    {
    #if CUTTER_COMP_ENABLE == 2
        return settings.flags.cc_chamfer_corner | (settings.flags.cc_lookahead_enable << 1);
    #else
        return settings.flags.cc_chamfer_corner;
    #endif
    }

    PROGMEM static const setting_detail_t ioport_settings[] = {
#if CUTTER_COMP_ENABLE == 2
        { Setting_CutterCompOptions, Group_General, "Cutter comp options", NULL, Format_Bitfield, "Chamfer corner,Enable lookahead", NULL, NULL, Setting_IsExtendedFn, set_options, get_options }
#else
        { Setting_CutterCompOptions, Group_General, "Cutter comp chamfer corner", NULL, Format_Bool, "Cutter comp chamfer corner", NULL, NULL, Setting_IsExtendedFn, set_options, get_options }
#endif
    };

    PROGMEM static const setting_descr_t ioport_settings_descr[] = {
#if CUTTER_COMP_ENABLE == 2
            { Setting_CutterCompOptions, "'Chamfer corner' controls default corner treatment behavior.\\n"
                                         "Enable the option for chamfer corner-treatment mode; leave it off to default to roll mode.\\n"
                                         "A P1 word on the G41/G42 entry block overrides the default for that command.\\n\\n"
                                         "'Enable lookahead' allow lookahead data for gouge checking.\\n"
                                         "When disabled, the system looks at the next move only.\\n"
                                         "When enabled, the system utilizes lookahead data to avoid gouging."}
#else
            { Setting_CutterCompOptions, "Controls default corner treatment behavior.\\n"
                                         "Enable the option for chamfer corner-treatment mode; leave it off to default to roll mode.\\n"
                                         "A P1 word on the G41/G42 entry block overrides the default for that command." }
#endif
    };

    static void onReportOptions (bool newopt)
    {
        on_report_options(newopt);

        if(!newopt)
            report_plugin("Cutter compensation", CUTTER_COMP_VERSION);
    }

    static inline void cc_mc_reset_runtime_state(const float *pos)
    {
        cc_mc_have_plan_data = false;
        cc_mc_active = false;
        cc_mc_pause_after_next_motion = false;
        cc_mc_output_paused = false;
        cc_mc_pending_dwell = 0.0f;

        for (int i = 0; i < N_AXIS; ++i)
            cc_mc_input_pos[i] = pos ? pos[i] : 0.0f;
    }

    // Sync cc_mc_input_pos with the current parser position.
    // Must be called when enabling comp, and can be called any time cc_mc_input_pos
    // may be stale (e.g. after rapids that bypass cc_mc_line_in).
    // This also clears transient bridge state so a restarted job cannot inherit
    // a pending synthetic pause from the previous run.
    static inline void cc_mc_sync_input_pos(const float *pos)
    {
        cc_mc_reset_runtime_state(pos);
    }

    static bool cc_mc_is_active(void)
    {
        return cc_mc_active;
    }

    // M70: Save current comp modal state
    static inline void cc_mc_save_modal_state(void)
    {
        cc_mc_saved_comp_side = cc_api_get_comp();
        cc_mc_saved_comp_mode = cc_api_get_mode();
        cc_mc_saved_state_valid = true;
    }

    // M71: Invalidate saved comp state
    static inline void cc_mc_invalidate_modal_state(void)
    {
        cc_mc_saved_state_valid = false;
    }

    // M72/M73 return: Restore comp state.
    // If comp is restored active, force STEADY so execution continues as if
    // compensation had not been canceled in between.
    static inline bool cc_mc_restore_modal_state(void)
    {
        if (!cc_mc_saved_state_valid)
            return false;

        comp_mode restore_mode = CC_CM_NONE;
        if (cc_mc_saved_comp_side != CC_COMP_OFF)
            restore_mode = CC_CM_STEADY;
        else
            restore_mode = cc_mc_saved_comp_mode;

        cc_api_restore_comp(cc_mc_saved_comp_side, restore_mode);
        return true;
    }

    static inline cc_status_code_t cc_mc_enqueue_pause_marker(float dwell)
    {
        move2d marker = {0};
        marker.type = CC_MOT_EMPTY;
        marker.pause_after = dwell;
        marker.valid = true;

        return cc_api_process_move(&marker);
    }

    static void cc_message(cc_status_code_t msgcode, msg_type_t severity, uint32_t lineNum)
    {
        const char *msg = "Unknown";
        switch (msgcode)
        {
        case cc_status_OK:
            return;
        case cc_status_ArcLtToolRad:
            msg = "Arc radius less than tool radius";
            break;
        case cc_status_InvalidMove:
            msg = "Invalid move";
            break;
        case cc_status_MoveTooShort:
            msg = "Move too short to compensate";
            break;
        case cc_status_ArcRadiusInconsistent:
            msg = "Arc radius inconsistent";
            break;
        case cc_status_UnresolvedGap:
            msg = "Unresolved gap between moves";
            break;
        case cc_status_InputBufferOverflow:
            msg = "Input buffer overflow";
            break;
        case cc_status_OutputBufferOverflow:
            msg = "Output buffer overflow";
            break;
        case cc_status_CompInCrossing:
            msg = "Crossing error: move in cutting area";
            break;
        case cc_status_CompOutCrossing:
            msg = "Crossing error: move out of cutting area";
            break;
        case cc_status_GlobalSelfIntersection:
            msg = "Global self intersection detected";
            break;
        }

        char formatted_msg[128] = {0};
        if (lineNum != 0)
        {
            snprintf(formatted_msg, sizeof(formatted_msg), "CC:%s at line %lu", msg, (unsigned long)lineNum);
            report_message(formatted_msg, (message_type_t)severity);
        }
        else
        {
            snprintf(formatted_msg, sizeof(formatted_msg), "CC:%s", msg);
            report_message(formatted_msg, (message_type_t)severity);
        }
        if (severity == CC_MSG_ERROR)
            system_set_exec_state_flag(EXEC_FEED_HOLD);
    }

    // Creates a move2d struct from the given grblHAL cutter compensation data.
    static inline move2d cc_mc_to_move2d(gc_ccomp_t cc,
                                         float *xyz,
                                         plan_line_data_t *pl_data,
                                         float *position,
                                         float *ijk,
                                         float radius,
                                         int32_t turns,
                                         bool is_arc)
    {
        move2d mv = {0};
        mv.p_0 = cc_v2(cc_mc_input_pos[plane.axis_0], cc_mc_input_pos[plane.axis_1]);
        mv.p_1 = cc_v2(xyz[plane.axis_0], xyz[plane.axis_1]);
        mv.z_0 = cc_mc_input_pos[plane.axis_linear];
        mv.z_1 = xyz[plane.axis_linear];
        mv.feed = pl_data ? pl_data->feed_rate : 0.0f;
        mv.lineNum = pl_data ? pl_data->line_number : 0;
        mv.compMode = (uint8_t)cc_api_get_mode();
        mv.valid = true;

        if (is_arc)
        {
            mv.type = CC_MOT_ARC;
            if (position)
            {
                mv.p_0 = cc_v2(position[plane.axis_0], position[plane.axis_1]);
                mv.z_0 = position[plane.axis_linear];
            }
            if (ijk)
            {
                mv.center = cc_v2(mv.p_0.x + ijk[plane.axis_0], mv.p_0.y + ijk[plane.axis_1]);
            }
            mv.radius = radius;
            mv.arcDir = (uint8_t)((turns >= 0) ? CC_ARC_CCW : CC_ARC_CW);
        }
        else
        {
            mv.type = ((pl_data && pl_data->condition.rapid_motion) ? CC_MOT_RAPID : CC_MOT_LINE);
        }

        cc_mc_input_pos[plane.axis_0] = xyz[plane.axis_0];
        cc_mc_input_pos[plane.axis_1] = xyz[plane.axis_1];
        cc_mc_input_pos[plane.axis_linear] = xyz[plane.axis_linear];
        return mv;
    }

    // replaces mc_line when cutter compensation is active. If compensation is not active, passes through to mc_line.
    status_code_t cc_mc_line_in(float *target, plan_line_data_t *pl_data)
    {
        if ((cc_mc_have_plan_data = pl_data != NULL))
            cc_mc_active_plan_data = *pl_data;

        if (cc.side == CComp_Off && cc_api_get_comp() == CC_COMP_OFF)
        {
            cc_mc_active = false;
            cc_mc_pause_after_next_motion = false;
            cc_mc_input_pos[plane.axis_0] = target[plane.axis_0];
            cc_mc_input_pos[plane.axis_1] = target[plane.axis_1];
            cc_mc_input_pos[plane.axis_linear] = target[plane.axis_linear];
            return core_mc_line(target, pl_data);
        }
        cc_mc_active = true;

        comp_side side = cc.side == CComp_Left ? CC_COMP_LEFT : (cc.side == CComp_Right ? CC_COMP_RIGHT : CC_COMP_OFF);
        comp_side current_side = cc_api_get_comp();
        bool turning_off = current_side != CC_COMP_OFF && side == CC_COMP_OFF;

        if (side != current_side || (side != CC_COMP_OFF && cc_api_get_mode() == CC_CM_NONE))
            cc_api_set_comp(side);

        move2d mv = cc_mc_to_move2d(cc, target, pl_data, NULL, NULL, 0.0f, 0, false);

        if (side != CC_COMP_OFF && mv.compMode == CC_CM_IN)
        {
            cc_units units = cc_api_get_units();
            float r = cc.radius;
            const char *corner_mode = cc_api_get_corner_treatment_mode() == CC_CTM_CHAMFER ? "Chamfer" : "Roll";
            char msg[96];
            snprintf(msg, sizeof(msg), "CC_On R=%.4f %s Corner=%s", r, units == CC_UNITS_INCH ? "in" : "mm", corner_mode);
            report_message(msg, Message_Info);
        }

        cc_status_code_t st = cc_api_process_move(&mv);
        if (st == cc_status_OK && turning_off) {
            if((st = cc_api_process_move(0)) == cc_status_OK)
                report_message("CC_Off", Message_Info);
        }

        return st == cc_status_OK ? Status_Handled : Status_CutterCompInvalid;
    }

    // replaces mc_arc when cutter compensation is active. If compensation is not active, passes through to mc_arc.
    static status_code_t cc_mc_arc_in(float *target, plan_line_data_t *pl_data, float *position, float *offset, float radius, plane_t plane, int32_t turns)
    {
        if ((cc_mc_have_plan_data = pl_data != NULL))
            cc_mc_active_plan_data = *pl_data;

        if (cc.side == CComp_Off && cc_api_get_comp() == CC_COMP_OFF)
        {
            cc_mc_active = false;
            cc_mc_pause_after_next_motion = false;
            cc_mc_input_pos[plane.axis_0] = target[plane.axis_0];
            cc_mc_input_pos[plane.axis_1] = target[plane.axis_1];
            cc_mc_input_pos[plane.axis_linear] = target[plane.axis_linear];
            return core_mc_arc(target, pl_data, position, offset, radius, plane, turns);
        }
        cc_mc_active = true;
        comp_side side = cc.side == CComp_Left ? CC_COMP_LEFT : (cc.side == CComp_Right ? CC_COMP_RIGHT : CC_COMP_OFF);
        comp_side current_side = cc_api_get_comp();
        bool turning_off = current_side != CC_COMP_OFF && side == CC_COMP_OFF;

        if (side != current_side || (side != CC_COMP_OFF && cc_api_get_mode() == CC_CM_NONE))
            cc_api_set_comp(side);

        move2d mv = cc_mc_to_move2d(cc, target, pl_data, position, offset, radius, turns, true);

        cc_status_code_t st = cc_api_process_move(&mv);
        if (st == cc_status_OK && turning_off) {
            st = cc_api_process_move(0);
        }

        return st == cc_status_OK ? Status_Handled : Status_CutterCompInvalid;
    }

    static bool cc_emit_via_mc(const move2d *mv)
    {
        // report_message("CC: cc_emit_via_mc", Message_Info);
        plan_line_data_t local_pl_data = {0};
        plan_line_data_t *pl_data = &local_pl_data;
        float target[N_AXIS] = {0};
        bool emitted_motion = false;

        if (!mv || !mv->valid)
            return true;

        if (mv->pause_after != 0.0f)
        {
            // Synthetic marker for a deferred pause or dwell after the next emitted motion.
            cc_mc_pause_after_next_motion = true;
            cc_mc_pending_dwell = 0.0f;
            if (mv->pause_after > 0.0f)
                cc_mc_pending_dwell = mv->pause_after;
            return true;
        }

        if (cc_mc_have_plan_data)
            local_pl_data = cc_mc_active_plan_data;

        local_pl_data.feed_rate = mv->feed;
        local_pl_data.condition.rapid_motion = (mv->type == CC_MOT_RAPID) ? 1 : 0;

        if (mv->type == CC_MOT_LINE || mv->type == CC_MOT_RAPID)
        {
            target[plane.axis_0] = mv->p_1.x;
            target[plane.axis_1] = mv->p_1.y;
            target[plane.axis_linear] = mv->z_1;
            core_mc_line(target, pl_data);
            emitted_motion = true;
        }
        else if (mv->type == CC_MOT_ARC)
        {
            target[plane.axis_0] = mv->p_1.x;
            target[plane.axis_1] = mv->p_1.y;
            target[plane.axis_linear] = mv->z_1;

            float position[N_AXIS] = {0};
            position[plane.axis_0] = mv->p_0.x;
            position[plane.axis_1] = mv->p_0.y;
            position[plane.axis_linear] = mv->z_0;

            float offset[3];
            offset[plane.axis_0] = mv->center.x - mv->p_0.x;
            offset[plane.axis_1] = mv->center.y - mv->p_0.y;
            offset[plane.axis_linear] = 0.0f;

            int32_t turns = (mv->arcDir == CC_ARC_CCW) ? 1 : -1;
            core_mc_arc(target, pl_data, position, offset, mv->radius, plane, turns);
            emitted_motion = true;
        }

        // Only a synthetic cutter-comp pause marker should pause here; grbl already pauses once per
        // gcode line natively, so reacting to sys.flags.single_block per emitted sub-move would
        // over-pause at corner/roll transitions within a single line.
        if (cc_mc_pause_after_next_motion && emitted_motion)
        {
            float dwell = cc_mc_pending_dwell;

            report_message("CC: Pausing after move", Message_Info);
            cc_mc_pause_after_next_motion = false;
            cc_mc_pending_dwell = 0.0f;

            if (dwell > 0.0f)
            {
                report_message("CC: Dwell...", Message_Info);
                mc_dwell(dwell);
                return true;
            }

            protocol_buffer_synchronize();
            system_set_exec_state_flag(EXEC_FEED_HOLD);
            protocol_execute_realtime();
        }

        return true;
    }

    // Trap G4
    FLASHMEM static status_code_t OnPreGcodeExecute (modal_groups_t *commands, parser_state_t *gc_state, parser_block_t *gc_block, spindle_t *spindle)
    {
        if(cc_mc_is_active() && gc_block->non_modal_command == NonModal_Dwell) {
            if(cc_mc_enqueue_pause_marker(gc_block->values.p) != cc_status_OK)
                return Status_CutterCompInvalid;

            gc_block->non_modal_command = NonModal_NoAction;
        }

        if(commands->G1)
            gc_get_plane_data(&plane, gc_block->modal.plane_select);

        return Status_Unhandled;
    }

    // Trap M0, M1 and M60
    FLASHMEM static status_code_t onProgramPaused (program_flow_t program_flow, bool check_mode)
    {
        if((on_program_paused == NULL || on_program_paused(program_flow, check_mode) == Status_Unhandled) &&
             !check_mode && program_flow != ProgramFlow_Running) {

            if(cc_mc_is_active())
                return cc_mc_enqueue_pause_marker(-1.0f) == cc_status_OK ? Status_Handled : Status_CutterCompInvalid;
        }
        return on_program_paused ? on_program_paused(program_flow, check_mode) : Status_Unhandled;
    }

    FLASHMEM void onModalStateAction (modal_state_action_t action, modal_groups_t commands, void *context)
    {
        if(on_modal_state_action)
            on_modal_state_action(action, commands, context);

        switch(action) {

            case ModalState_Save:
                report_message("cutter_comp_save_state", Message_Plain);
                cc_mc_save_modal_state();
                break;

            case ModalState_Invalidate:
                cc_mc_invalidate_modal_state();
                report_message("cutter_comp_invalidate_state", Message_Plain);
                break;

            case ModalState_Restore:
                // commands.G7 is set on change, this call will be followed by cc_enable with restored values (for now)
                report_message("cutter_comp_restore_state", Message_Plain);
                //The state of the comp engine should now be the same as it was.
                cc_mc_restore_modal_state();
                break;

            default:
                break;
        }
    }

    FLASHMEM void cc_init (void)
    {
        static bool init_ok = false;
        static setting_details_t setting_details = {
            .is_core = true,
            .settings = ioport_settings,
            .n_settings = sizeof(ioport_settings) / sizeof(setting_detail_t),
            .descriptions = ioport_settings_descr,
            .n_descriptions = sizeof(ioport_settings_descr) / sizeof(setting_descr_t),
            .save = settings_write_global
        };

        if(!init_ok) {

            init_ok = true;
            settings_register(&setting_details);

            core_mc_line = grbl.mc_line;
            grbl.mc_line = cc_mc_line_in;

            core_mc_arc  = grbl.mc_arc;
            grbl.mc_arc = cc_mc_arc_in;

            on_program_paused = grbl.on_program_paused;
            grbl.on_program_paused = onProgramPaused;

            on_modal_state_action = grbl.on_modal_state_action;
            grbl.on_modal_state_action = onModalStateAction;

            on_state_change = grbl.on_state_change;
            grbl.on_state_change = cc_on_state_change;

            // This needs to be done after the other callbacks are set up, to ensure proper chaining of control signal changes.
            // cc_prev_on_control_signals_changed = grbl.on_control_signals_changed;
            // grbl.on_control_signals_changed = cc_on_control_signals_changed;

            on_report_options = grbl.on_report_options;
            grbl.on_report_options = onReportOptions;

            on_pre_gcode_execute = grbl.on_pre_gcode_execute;
            grbl.on_pre_gcode_execute = OnPreGcodeExecute;
        }
    }

    FLASHMEM static inline comp_side cutter_comp_side_to_core (ccomp_mode_t side)
    {
        return side == CComp_Left ? CC_COMP_LEFT : (side == CComp_Right ? CC_COMP_RIGHT : CC_COMP_OFF);
    }

    FLASHMEM static inline void cutter_comp_apply_settings (void)
    {
#if CUTTER_COMP_ENABLE == 2
        cc_api_set_lookahead_enabled(settings.flags.cc_lookahead_enable);
#endif
        cc_api_set_corner_treatment_mode(settings.flags.cc_chamfer_corner ? CC_CTM_CHAMFER : CC_CTM_ROLL);
    }

    FLASHMEM bool cc_enable (gc_ccomp_t *comp_data, plane_t cc_plane, coord_data_t *position)
    {
        memcpy(&cc, comp_data, sizeof(gc_ccomp_t));

        if(comp_data->side) {
            plane = cc_plane;
            cc_api_init(comp_data->radius, CC_UNITS_MM, cc_emit_via_mc, cc_message);
            cutter_comp_apply_settings();
            cc_mc_sync_input_pos(position->values); // Ensure start pos is current, not stale
            cc_api_set_comp(cutter_comp_side_to_core(comp_data->side));
        } else {
            cc_api_process_move(0);
            // Pause once here, right after the final compensated exit move is flushed, so a standalone
            // G40 cancel stops immediately rather than deferring to the next (uncompensated) line.
            if (sys.flags.single_block) {
                protocol_buffer_synchronize();
                system_set_exec_state_flag(EXEC_FEED_HOLD);
                protocol_execute_realtime();
            }
            cc_api_set_comp(CC_COMP_OFF);
            cc_mc_reset_runtime_state(position->values);
        }

        return true;
    }

#endif
#ifdef __cplusplus
}
#endif
#endif
