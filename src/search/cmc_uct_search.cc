#include "cmc_uct_search.h"

using namespace std;

/******************************************************************
                        Initialization
******************************************************************/

void CMCUCTSearch::initializeDecisionNodeChild(CMCUCTNode* node,
                                              unsigned int const& actionIndex,
                                              double const& initialQValue) {
    node->children[actionIndex] = getSearchNode();
    node->children[actionIndex]->futureReward =
        (double) numberOfInitialVisits * (double) remainingConsideredSteps() *
        initialQValue;
    node->children[actionIndex]->numberOfVisits = numberOfInitialVisits;

    node->numberOfVisits += numberOfInitialVisits;
    node->futureReward =
        std::max(node->futureReward, node->children[actionIndex]->futureReward);
    //cout<<"MC futReward:" <<node->futureReward<<endl;
    // cout << "initialized child ";
    // SearchEngine::actionStates[actionIndex].printCompact(cout);
    // cout << " with remaining steps " << remainingConsideredSteps() << " and initialQValue " << initialQValue << endl;
    // node->children[actionIndex]->print(cout);
    // cout << endl;
}

/******************************************************************
                         Outcome selection
******************************************************************/

CMCUCTNode* CMCUCTSearch::selectOutcome(CMCUCTNode* node, 
                                      PDState& nextState,
                                      int& varIndex) {
    if (node->children.empty()) {
        node->children.resize(
                SearchEngine::probabilisticCPFs[varIndex]->getDomainSize(),
                NULL);
    }

    int childIndex = (int)nextState.sample(varIndex);

    if (!node->children[childIndex]) {
        node->children[childIndex] = getSearchNode();
    }
    return node->children[childIndex];
}

/******************************************************************
                          Backup functions
******************************************************************/

void CMCUCTSearch::backupDecisionNode(CMCUCTNode* node, double const& immReward,
                                     double const& /*futReward*/) {
    node->immediateReward += immReward;
    ++node->numberOfVisits;

    //Calculate the weighted average and confidence of the childrens rewards.
    double weightedAvg = 0.0;
    int visits = 0;
    double oldCI = 0.0;

    for (unsigned int childIndex = 0; childIndex < node->children.size(); ++childIndex) {
    	if (node->children[childIndex]) {
		int childVisits = node->children[childIndex]->numberOfVisits;
		double childFutureRewards = node->children[childIndex]->getExpectedRewardEstimate();
		double childCI = node->children[childIndex]->ci;

		weightedAvg += childFutureRewards * (double)childVisits;
		oldCI += (double)(childVisits - 1) * childCI + (double)childVisits * pow(childFutureRewards, 2); 
		visits += childVisits;
        }
    }

    weightedAvg /= (double)max(visits, 1);
    oldCI = oldCI - (double)visits * pow(weightedAvg, 2);
    oldCI = MathUtils::doubleIsEqual(oldCI, 0.0) ? 0.0 : oldCI;
    oldCI = oldCI / max(visits - 1.0, 1.0);

    // Propagate values from best child
    node->futureReward = weightedAvg;
    node->ci = oldCI;
    bool updated = false;
    for (unsigned int childIndex = 0; childIndex < node->children.size(); ++childIndex) {
        if (node->children[childIndex]) {
	    if (MathUtils::doubleIsGreater(node->children[childIndex]->getExpectedRewardEstimate(), node->futureReward) &&
		MathUtils::doubleIsSmaller(node->children[childIndex]->ci, oldCI)) {
		node->futureReward = node->children[childIndex]->getExpectedRewardEstimate();
		node->ci = node->children[childIndex]->ci;
		updated = true;
	    }
        }
    }
    
    tests++;
    if (updated)
	updates++;
}

void CMCUCTSearch::backupChanceNode(CMCUCTNode* node, double const& /*futReward*/) {
    assert(MathUtils::doubleIsEqual(node->immediateReward, 0.0));

    ++node->numberOfVisits;

    // Propagate values from children
    node->futureReward = 0.0;
    node->ci = 0.0;
    double visits = 0.0;
    for (unsigned int i = 0; i < node->children.size(); ++i) {
        if (node->children[i]) {
	    double childCI = node->children[i]->ci;
	    double childFutureRewards = node->children[i]->getExpectedRewardEstimate();
	    double childVisits = node->children[i]->numberOfVisits;

	    visits += childVisits;
            node->futureReward += childVisits * childFutureRewards;
	    node->ci += (childVisits - 1) * childCI + childVisits * pow(childFutureRewards, 2);
        }
    }

    node->futureReward /= visits;
    node->ci = node->ci - visits * pow(node->futureReward, 2);
    node->ci = MathUtils::doubleIsEqual(node->ci, 0.0) ? 0.0 : node->ci;
    node->ci = node->ci / (visits - 1.0); 
}
