#include "mscp_heuristic.h"

#include "../merge_and_shrink/factored_transition_system.h"
#include "../merge_and_shrink/transition_system.h"
#include "../merge_and_shrink/merge_and_shrink_algorithm.h"
#include "../merge_and_shrink/merge_and_shrink_representation.h"
#include "../merge_and_shrink/fts_factory.h"
#include "../merge_and_shrink/distances.h"

#include "../option_parser.h"
#include "../plugin.h"

#include "../task_utils/task_properties.h"
#include "../utils/logging.h"

#include <cassert>
#include <iostream>
#include <limits>
#include <set>
#include <string>

using namespace std;

namespace mscp_heuristic {
MSCPHeuristic::MSCPHeuristic(const Options &opts)
    : Heuristic(opts),
      log(utils::get_log_from_options(opts)),
      fts(merge_and_shrink::create_factored_transition_system(task_proxy, true, true, log)),
      treshold(opts.get<int>("treshold")) {
      while (fts.get_num_active_entries() > 1) {
          // TEST PRINT INITIAL GOAL
          cout << " -- FTS size = " << fts.get_num_active_entries() << endl;

//          for (int i = 0; i < fts.get_num_active_entries(); ++i) {
//              const merge_and_shrink::TransitionSystem &tr_sys = fts.get_transition_system(i);
//              unique_ptr <merge_and_shrink::Distances> d = utils::make_unique_ptr<merge_and_shrink::Distances>(
//                      tr_sys);
//              d->compute_distances(false, true, log);
//              int idist = d->get_goal_distance(tr_sys.get_init_state());
//              cout << "Initial goal distance of factor " << i << " is: " << idist << endl;
//          }
          // END TEST PRINT

          int quality = -1;
          string best_pair = "";
          int best_i = -1;
          int best_j = -1;

          for (int i = 0; i < fts.get_num_active_entries() - 1; i++)
              for (int j = i + 1; j < fts.get_num_active_entries(); j++) {
                  if (fts.is_active(i) && fts.is_active(j)){
                      // get the transition systems at indexes i and j
                      const merge_and_shrink::TransitionSystem &tr_sys0 = fts.get_transition_system(i);
                      const merge_and_shrink::TransitionSystem &tr_sys1 = fts.get_transition_system(j);
                      // calculate the distances for both
                      unique_ptr <merge_and_shrink::Distances> dist0 = utils::make_unique_ptr<merge_and_shrink::Distances>(
                              tr_sys0);
                      unique_ptr <merge_and_shrink::Distances> dist1 = utils::make_unique_ptr<merge_and_shrink::Distances>(
                              tr_sys1);
                      dist0->compute_distances(false, true, log);
                      dist1->compute_distances(false, true, log);
                      idist0 = dist0->get_goal_distance(tr_sys0.get_init_state());
                      idist1 = dist1->get_goal_distance(tr_sys1.get_init_state());

                      // compute the merged system
                      const unique_ptr <merge_and_shrink::TransitionSystem> merged_tr_sys = fts.merge_and_keep(i, j,
                                                                                                               log);
                      // calculate the distance for the merged system
                      unique_ptr <merge_and_shrink::Distances> dist = utils::make_unique_ptr<merge_and_shrink::Distances>(
                              *merged_tr_sys);
                      dist->compute_distances(false, true, log);
                      idist = dist->get_goal_distance(merged_tr_sys->get_init_state());

                      // TEST PRINT DISTANCES
//                      cout << " --------------------------------------------------------" << endl;
//                      cout << " Current pair: " << std::to_string(i) + ", " + std::to_string(j) << endl;
//                      cout << " Initial goal distance of factor " << i << " is: " << idist0 << endl;
//                      cout << " Initial goal distance of factor " << j << " is: " << idist1 << endl;
//                      cout << " Initial goal distance of merged system is: " << idist << endl;
                      // END TEST PRINT DISTANCES

                      if (quality < idist - (idist0 + idist1)) {
                          quality = idist - (idist0 + idist1);
                          best_pair = std::to_string(i) + ", " + std::to_string(j);
                          best_i = i;
                          best_j = j;
                      }
                  }
              }

          cout << " --------------------------------------------------------" << endl;
          cout << " -- Best pair to merge is: " << best_pair << " with merge quality: " << quality << endl;

          int merge_index = -1;

          if (quality > treshold) {
              cout << " ---- Decision: Merge!" << endl;
              merge_index = fts.merge(best_i, best_j, log);
          }
          else {
              cout << " ---- Decision: Don't merge!" << endl;
          }

          if (merge_index > -1){
              cout << " --------------------------------------------------------" << endl;
              cout << " NEXT ITERATION" << endl << endl;
          }
          else {
              cout << " --------------------------------------------------------" << endl;
              exit(0);
          }
      }
}

bool MSCPHeuristic::dead_ends_are_reliable() const {
    return true;
}

int MSCPHeuristic::compute_heuristic(const State &ancestor_state) {
    State state = convert_ancestor_state(ancestor_state);
    if (task_properties::is_goal_state(task_proxy, state)) {
        return 0;
    }

    return 1;
}

static shared_ptr<Heuristic> _parse(OptionParser &parser) {
    parser.document_synopsis("mscp heuristic", "");
    parser.add_option<int>("treshold", "subset size", "0");
    utils::add_log_options_to_parser(parser);
    Heuristic::add_options_to_parser(parser);
    Options opts = parser.parse();
    if (parser.dry_run())
        return nullptr;
    else
        return make_shared<MSCPHeuristic>(opts);
}

static Plugin<Evaluator> _plugin("mscp", _parse);
}
