#ifndef MS_CP_MSCP_HEURISTIC_H
#define MS_CP_MSCP_HEURISTIC_H

#include "../heuristic.h"
#include "../merge_and_shrink/factored_transition_system.h"
#include "../merge_and_shrink/transition_system.h"
#include "../merge_and_shrink/distances.h"

namespace options {
class Options;
}

namespace mscp_heuristic {

class MSCPHeuristic : public Heuristic {
    mutable utils::LogProxy log;
    merge_and_shrink::FactoredTransitionSystem fts;
    int testint;
    int idist;
    int idist0;
    int idist1;

protected:
    virtual int compute_heuristic(const State &ancestor_state) override;

public:
    explicit MSCPHeuristic(const options::Options &opts);

    virtual bool dead_ends_are_reliable() const override;
};
}

#endif
