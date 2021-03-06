#include "mc_uct_search.h"

using namespace std;

/******************************************************************
                        Initialization
******************************************************************/

void MCUCTSearch::initializeDecisionNodeChild(MCUCTNode* node,
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

MCUCTNode* MCUCTSearch::selectOutcome(MCUCTNode* node, 
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

void MCUCTSearch::backupDecisionNode(MCUCTNode* node, double const& immReward,
                                     double const& futReward) {
    node->immediateReward += immReward;
    node->futureReward += futReward;
    ++node->numberOfVisits;
    //cout<<"MC futReward:" <<node->futureReward<<endl;
}

void MCUCTSearch::backupChanceNode(MCUCTNode* node, double const& futReward) {
    assert(MathUtils::doubleIsEqual(node->immediateReward, 0.0));

    node->futureReward += futReward;
    ++node->numberOfVisits;
}
