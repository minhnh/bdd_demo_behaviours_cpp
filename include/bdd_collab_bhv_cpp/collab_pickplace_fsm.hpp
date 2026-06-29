/*
 * This is an auto-generated file. Do not edit it directly.
 *
 * FSM: collab_pickplace
 * FSM Description: FSM for a collaborative pick&place task with human
 *
 * -----------------------------------------------------
 * Usage example:
 * -----------------------------------------------------

#include "coord2b/functions/event_loop.h"
#include "coord2b/functions/fsm.h"
#include "collab_pickplace.hpp"

struct user_data {

};

void yyyy_behavior(struct user_data *userData, struct events *eventData) {
    // ... do something

    produce_event(eventData, collab_pickplace::E_ZZZZ);
}

void fsm_behavior(struct events *eventData, struct user_data *userData) {
    if (consume_event(eventData, collab_pickplace::E_XXXX)) {
        yyyy_behavior(userData, eventData);
    }
    ...
}

int main() {

    struct user_data userData = {};
    struct fsm_nbx *fsm = collab_pickplace::create_fsm();
    if (!fsm) return 1;

    while (true) {
        produce_event(fsm->eventData, collab_pickplace::E_STEP);

        // run state machine, event loop
        fsm_behavior(fsm->eventData, &userData);
        fsm_step_nbx(fsm);
        reconfig_event_buffers(fsm->eventData);
    }

    collab_pickplace::destroy_fsm(fsm);
    return 0;
}

 * -----------------------------------------------------
 */

#ifndef COLLAB_PICKPLACE_HPP
#define COLLAB_PICKPLACE_HPP

#include "coord2b/types/fsm.h"
#include "coord2b/types/event_loop.h"
#include <new>


namespace collab_pickplace {

struct fsm_nbx *create_fsm();
void            destroy_fsm(struct fsm_nbx *fsm);

// sm states
enum e_states {
    S_START = 0,
    S_IDLE,
    S_TOUCH_TABLE,
    S_SLIDE,
    S_GRASP,
    S_COLLAB_MOVE,
    S_RELEASE,
    S_RECOVER,
    S_EXIT,
    NUM_STATES
};

static constexpr const char *STATE_URIS[NUM_STATES] = {
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/S_START",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/S_IDLE",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/S_TOUCH_TABLE",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/S_SLIDE",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/S_GRASP",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/S_COLLAB_MOVE",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/S_RELEASE",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/S_RECOVER",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/S_EXIT",
};

// sm events
enum e_events {
    E_PICK_APPROACH_START = 0,
    E_NEW_GOAL,
    E_TABLE_TOUCHED,
    E_OBJ_REACHED,
    E_GRASP_DONE,
    E_PLACE_REACHED,
    E_RELEASE_DONE,
    E_RECOVER_DONE,
    E_GOAL_CANCELLED,
    E_GOAL_FINISHED,
    E_STOP,
    E_STEP,
    NUM_EVENTS
};

static constexpr const char *EVENT_URIS[NUM_EVENTS] = {
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/E_PICK_APPROACH_START",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/E_NEW_GOAL",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/E_TABLE_TOUCHED",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/E_OBJ_REACHED",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/E_GRASP_DONE",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/E_PLACE_REACHED",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/E_RELEASE_DONE",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/E_RECOVER_DONE",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/E_GOAL_CANCELLED",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/E_GOAL_FINISHED",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/E_STOP",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/E_STEP",
};

// sm transitions
enum e_transitions {
    T_START_IDLE = 0,
    T_IDLE_SELF,
    T_IDLE_TOUCH,
    T_IDLE_EXIT,
    T_TOUCH_SELF,
    T_TOUCH_SLIDE,
    T_TOUCH_RECOVER,
    T_SLIDE_SLIDE,
    T_SLIDE_GRASP,
    T_SLIDE_RECOVER,
    T_GRASP_SELF,
    T_GRASP_COL_MOVE,
    T_COL_MOVE_SELF,
    T_COL_MOVE_RELEASE,
    T_RELEASE_SELF,
    T_RELEASE_RECOVER,
    T_RECOVER_SELF,
    T_RECOVER_IDLE,
    NUM_TRANSITIONS
};

static constexpr const char *TRANSITION_URIS[NUM_TRANSITIONS] = {
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/T_START_IDLE",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/T_IDLE_SELF",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/T_IDLE_TOUCH",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/T_IDLE_EXIT",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/T_TOUCH_SELF",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/T_TOUCH_SLIDE",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/T_TOUCH_RECOVER",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/T_SLIDE_SLIDE",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/T_SLIDE_GRASP",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/T_SLIDE_RECOVER",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/T_GRASP_SELF",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/T_GRASP_COL_MOVE",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/T_COL_MOVE_SELF",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/T_COL_MOVE_RELEASE",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/T_RELEASE_SELF",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/T_RELEASE_RECOVER",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/T_RECOVER_SELF",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/T_RECOVER_IDLE",
};

// sm reactions
enum e_reactions {
    R_E_NEW_GOAL = 0,
    R_E_TABLE_TOUCHED,
    R_E_OBJ_REACHED,
    R_E_GRASP_DONE,
    R_E_PLACE_REACHED,
    R_E_RELEASE_DONE,
    R_E_RECOVER_DONE,
    R_E_CANCEL_TOUCH,
    R_E_CANCEL_SLIDE,
    R_E_CANCEL_COL_MOVE,
    R_E_STOP_IDLE,
    R_E_STEP_START,
    R_E_STEP_IDLE,
    R_E_STEP_TOUCH,
    R_E_STEP_SLIDE,
    R_E_STEP_GRASP,
    R_E_STEP_COL_MOVE,
    R_E_STEP_RELEASE,
    R_E_STEP_RECOVER,
    NUM_REACTIONS
};

static constexpr const char *REACTION_URIS[NUM_REACTIONS] = {
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/R_E_NEW_GOAL",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/R_E_TABLE_TOUCHED",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/R_E_OBJ_REACHED",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/R_E_GRASP_DONE",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/R_E_PLACE_REACHED",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/R_E_RELEASE_DONE",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/R_E_RECOVER_DONE",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/R_E_CANCEL_TOUCH",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/R_E_CANCEL_SLIDE",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/R_E_CANCEL_COL_MOVE",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/R_E_STOP_IDLE",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/R_E_STEP_START",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/R_E_STEP_IDLE",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/R_E_STEP_TOUCH",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/R_E_STEP_SLIDE",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/R_E_STEP_GRASP",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/R_E_STEP_COL_MOVE",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/R_E_STEP_RELEASE",
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/R_E_STEP_RECOVER",
};

inline struct fsm_nbx *create_fsm()
{

    struct fsm_nbx *fsm = new (std::nothrow) fsm_nbx{ .numReactions      = NUM_REACTIONS,
                                                      .numTransitions    = NUM_TRANSITIONS,
                                                      .numStates         = NUM_STATES,
                                                      .states            = nullptr,
                                                      .startStateIndex   = S_START,
                                                      .endStateIndex     = S_EXIT,
                                                      .currentStateIndex = S_START,
                                                      .eventData         = nullptr,
                                                      .reactions         = nullptr,
                                                      .transitions       = nullptr };
    if (!fsm) return nullptr;

    // sm states
    struct state *states = new (std::nothrow) state[NUM_STATES]{
        { .name = "S_start" },   { .name = "S_idle" },    { .name = "S_touch_table" },
        { .name = "S_slide" },   { .name = "S_grasp" },   { .name = "S_collab_move" },
        { .name = "S_release" }, { .name = "S_recover" }, { .name = "S_exit" }
    };

    // sm transition table
    struct transition *transitions =
      new (std::nothrow) transition[NUM_TRANSITIONS]{ {
                                                        .startStateIndex = S_START,
                                                        .endStateIndex   = S_IDLE,
                                                      },
                                                      {
                                                        .startStateIndex = S_IDLE,
                                                        .endStateIndex   = S_IDLE,
                                                      },
                                                      {
                                                        .startStateIndex = S_IDLE,
                                                        .endStateIndex   = S_TOUCH_TABLE,
                                                      },
                                                      {
                                                        .startStateIndex = S_IDLE,
                                                        .endStateIndex   = S_EXIT,
                                                      },
                                                      {
                                                        .startStateIndex = S_TOUCH_TABLE,
                                                        .endStateIndex   = S_TOUCH_TABLE,
                                                      },
                                                      {
                                                        .startStateIndex = S_TOUCH_TABLE,
                                                        .endStateIndex   = S_SLIDE,
                                                      },
                                                      {
                                                        .startStateIndex = S_TOUCH_TABLE,
                                                        .endStateIndex   = S_RECOVER,
                                                      },
                                                      {
                                                        .startStateIndex = S_SLIDE,
                                                        .endStateIndex   = S_SLIDE,
                                                      },
                                                      {
                                                        .startStateIndex = S_SLIDE,
                                                        .endStateIndex   = S_GRASP,
                                                      },
                                                      {
                                                        .startStateIndex = S_SLIDE,
                                                        .endStateIndex   = S_RECOVER,
                                                      },
                                                      {
                                                        .startStateIndex = S_GRASP,
                                                        .endStateIndex   = S_GRASP,
                                                      },
                                                      {
                                                        .startStateIndex = S_GRASP,
                                                        .endStateIndex   = S_COLLAB_MOVE,
                                                      },
                                                      {
                                                        .startStateIndex = S_COLLAB_MOVE,
                                                        .endStateIndex   = S_COLLAB_MOVE,
                                                      },
                                                      {
                                                        .startStateIndex = S_COLLAB_MOVE,
                                                        .endStateIndex   = S_RELEASE,
                                                      },
                                                      {
                                                        .startStateIndex = S_RELEASE,
                                                        .endStateIndex   = S_RELEASE,
                                                      },
                                                      {
                                                        .startStateIndex = S_RELEASE,
                                                        .endStateIndex   = S_RECOVER,
                                                      },
                                                      {
                                                        .startStateIndex = S_RECOVER,
                                                        .endStateIndex   = S_RECOVER,
                                                      },
                                                      {
                                                        .startStateIndex = S_RECOVER,
                                                        .endStateIndex   = S_IDLE,
                                                      } };

    // sm reaction table
    struct event_reaction *reactions = new (std::nothrow) event_reaction[NUM_REACTIONS]{
        {
          .conditionEventIndex = E_NEW_GOAL,
          .transitionIndex     = T_IDLE_TOUCH,
          .numFiredEvents      = 1,
          .firedEventIndices   = new unsigned int[1]{ E_PICK_APPROACH_START },
        },
        {
          .conditionEventIndex = E_TABLE_TOUCHED,
          .transitionIndex     = T_TOUCH_SLIDE,
          .numFiredEvents      = 0,
          .firedEventIndices   = nullptr,
        },
        {
          .conditionEventIndex = E_OBJ_REACHED,
          .transitionIndex     = T_SLIDE_GRASP,
          .numFiredEvents      = 0,
          .firedEventIndices   = nullptr,
        },
        {
          .conditionEventIndex = E_GRASP_DONE,
          .transitionIndex     = T_GRASP_COL_MOVE,
          .numFiredEvents      = 0,
          .firedEventIndices   = nullptr,
        },
        {
          .conditionEventIndex = E_PLACE_REACHED,
          .transitionIndex     = T_COL_MOVE_RELEASE,
          .numFiredEvents      = 0,
          .firedEventIndices   = nullptr,
        },
        {
          .conditionEventIndex = E_RELEASE_DONE,
          .transitionIndex     = T_RELEASE_RECOVER,
          .numFiredEvents      = 0,
          .firedEventIndices   = nullptr,
        },
        {
          .conditionEventIndex = E_RECOVER_DONE,
          .transitionIndex     = T_RECOVER_IDLE,
          .numFiredEvents      = 1,
          .firedEventIndices   = new unsigned int[1]{ E_GOAL_FINISHED },
        },
        {
          .conditionEventIndex = E_GOAL_CANCELLED,
          .transitionIndex     = T_TOUCH_RECOVER,
          .numFiredEvents      = 0,
          .firedEventIndices   = nullptr,
        },
        {
          .conditionEventIndex = E_GOAL_CANCELLED,
          .transitionIndex     = T_SLIDE_RECOVER,
          .numFiredEvents      = 0,
          .firedEventIndices   = nullptr,
        },
        {
          .conditionEventIndex = E_GOAL_CANCELLED,
          .transitionIndex     = T_COL_MOVE_RELEASE,
          .numFiredEvents      = 0,
          .firedEventIndices   = nullptr,
        },
        {
          .conditionEventIndex = E_STOP,
          .transitionIndex     = T_IDLE_EXIT,
          .numFiredEvents      = 0,
          .firedEventIndices   = nullptr,
        },
        {
          .conditionEventIndex = E_STEP,
          .transitionIndex     = T_START_IDLE,
          .numFiredEvents      = 0,
          .firedEventIndices   = nullptr,
        },
        {
          .conditionEventIndex = E_STEP,
          .transitionIndex     = T_IDLE_SELF,
          .numFiredEvents      = 0,
          .firedEventIndices   = nullptr,
        },
        {
          .conditionEventIndex = E_STEP,
          .transitionIndex     = T_TOUCH_SELF,
          .numFiredEvents      = 0,
          .firedEventIndices   = nullptr,
        },
        {
          .conditionEventIndex = E_STEP,
          .transitionIndex     = T_SLIDE_SLIDE,
          .numFiredEvents      = 0,
          .firedEventIndices   = nullptr,
        },
        {
          .conditionEventIndex = E_STEP,
          .transitionIndex     = T_GRASP_SELF,
          .numFiredEvents      = 0,
          .firedEventIndices   = nullptr,
        },
        {
          .conditionEventIndex = E_STEP,
          .transitionIndex     = T_COL_MOVE_SELF,
          .numFiredEvents      = 0,
          .firedEventIndices   = nullptr,
        },
        {
          .conditionEventIndex = E_STEP,
          .transitionIndex     = T_RELEASE_SELF,
          .numFiredEvents      = 0,
          .firedEventIndices   = nullptr,
        },
        {
          .conditionEventIndex = E_STEP,
          .transitionIndex     = T_RECOVER_SELF,
          .numFiredEvents      = 0,
          .firedEventIndices   = nullptr,
        }
    };

    if (!states || !transitions || !reactions) {
        delete[] states;
        delete[] transitions;
        delete[] reactions;
        delete fsm;
        return nullptr;
    }

    for (unsigned int i = 0; i < NUM_REACTIONS; ++i) {
        if (reactions[i].numFiredEvents > 0 && !reactions[i].firedEventIndices) {
            for (unsigned int j = 0; j < NUM_REACTIONS; ++j) {
                delete[] reactions[j].firedEventIndices;
            }
            delete[] reactions;
            delete[] transitions;
            delete[] states;
            delete fsm;
            return nullptr;
        }
    }

    // sm event data
    struct events *eventData     = new (std::nothrow) events{};
    _Bool         *currentEvents = new (std::nothrow) _Bool[NUM_EVENTS]{ false };
    _Bool         *futureEvents  = new (std::nothrow) _Bool[NUM_EVENTS]{ false };
    if (!eventData || !currentEvents || !futureEvents) {
        delete[] states;
        delete[] transitions;
        if (reactions) {
            for (unsigned int i = 0; i < NUM_REACTIONS; ++i) {
                delete[] reactions[i].firedEventIndices;
            }
        }
        delete[] reactions;
        delete[] currentEvents;
        delete[] futureEvents;
        delete eventData;
        delete fsm;
        return nullptr;
    }
    eventData->numEvents     = NUM_EVENTS;
    eventData->currentEvents = currentEvents;
    eventData->futureEvents  = futureEvents;

    // sm fsm struct
    fsm->states      = states;
    fsm->eventData   = eventData;
    fsm->reactions   = reactions;
    fsm->transitions = transitions;

    return fsm;
}

inline void destroy_fsm(struct fsm_nbx *fsm)
{
    if (!fsm) return;
    if (fsm->reactions) {
        for (unsigned int i = 0; i < fsm->numReactions; ++i) {
            delete[] fsm->reactions[i].firedEventIndices;
            fsm->reactions[i].firedEventIndices = nullptr;
            fsm->reactions[i].numFiredEvents    = 0;
        }
    }
    if (fsm->eventData) {
        delete[] fsm->eventData->currentEvents;
        delete[] fsm->eventData->futureEvents;
        delete fsm->eventData;
        fsm->eventData = nullptr;
    }
    delete[] fsm->reactions;
    delete[] fsm->transitions;
    delete[] fsm->states;
    delete fsm;
}

} // namespace collab_pickplace

#endif // COLLAB_PICKPLACE_HPP