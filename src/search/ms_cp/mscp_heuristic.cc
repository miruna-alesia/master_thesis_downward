#include "mscp_heuristic.h"

#include "../merge_and_shrink/factored_transition_system.h"
#include "../merge_and_shrink/transition_system.h"
#include "../merge_and_shrink/merge_and_shrink_algorithm.h"
#include "../merge_and_shrink/merge_and_shrink_representation.h"
#include "../merge_and_shrink/fts_factory.h"

#include "../option_parser.h"
#include "../plugin.h"

#include "../task_utils/task_properties.h"
#include "../utils/logging.h"

#include <cassert>
#include <iostream>
#include <limits>
#include <set>

// TO DO:
// GET FIRST 2 FACTORS IN FTS
// MERGE THEM
// CALCULATE COST PARTITIONING
// COMPARE

using namespace std;

namespace mscp_heuristic {
MSCPHeuristic::MSCPHeuristic(const Options &opts)
    : Heuristic(opts),
      log(utils::get_log_from_options(opts)),
      fts(merge_and_shrink::create_factored_transition_system(task_proxy, true, true, log)),
      testint(opts.get<int>("testint")) {
//    merge_and_shrink::FactoredTransitionSystem
//    fts = merge_and_shrink::create_factored_transition_system(
//                    task_proxy,
//                    true,
//                    true,
//                    log);

//    merge_and_shrink::MergeAndShrinkAlgorithm algorithm(opts);
//    merge_and_shrink::FactoredTransitionSystem fts = algorithm.build_factored_transition_system(task_proxy);
      test_fct = fts.get_num_active_entries();

      // cost partitioning classes work with this, not fts
      const merge_and_shrink::TransitionSystem &tr_sys0 = fts.get_transition_system(0);
      const merge_and_shrink::TransitionSystem &tr_sys1 = fts.get_transition_system(1);

      // maybe get merging like this? confusing function
      const merge_and_shrink::TransitionSystem merged_tr_sys = *(fts.merge_and_keep(0,1,log).get());

//      unique_ptr<merge_and_shrink::Distances> dist = utils::make_unique_ptr<merge_and_shrink::Distances>(merged_tr_sys);
//      dist->compute_distances(false, true, log);
}

bool MSCPHeuristic::dead_ends_are_reliable() const {
    return true;
}

//int test(merge_and_shrink::FactoredTransitionSystem &fts) {
//    return fts.get_size;
//}


int MSCPHeuristic::compute_heuristic(const State &ancestor_state) {
    State state = convert_ancestor_state(ancestor_state);
    if (task_properties::is_goal_state(task_proxy, state)) {
        return 0;
    }

//    int cp_val = CostPartitioning::compute_value(ancestor_state);
//
//    // compare cp_val with merge index?
//    // what is merge index after all?
//
//    int heuristic;
//
//    if (cp_val > merge_index) {
//        heuristic = cp_val;
//        cout << "+++++++++++++++++++ cp_val = " << cp_val << endl;
//    }
//    else {
//        heuristic = merge_index;
//        cout << "+++++++++++++++++++ merge_index = " << merge_index << endl;
//    }

    return 1;
}


static shared_ptr<Heuristic> _parse(OptionParser &parser) {
    parser.document_synopsis("mscp heuristic", "");
    parser.add_option<int>("testint", "subset size", "2", Bounds("1", "infinity"));
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
