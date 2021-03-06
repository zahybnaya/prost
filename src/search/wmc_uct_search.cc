#include "wmc_uct_search.h"

using namespace std;

/******************************************************************
                     Initialization of Nodes
*******************************************************************/

void WMCUCTSearch::initializeDecisionNodeChild(WMCUCTNode* node,
                                              unsigned int const& actionIndex,
                                              double const& initialQValue) {
    node->children[actionIndex] = getWMCUCTNode(1.0);
    node->children[actionIndex]->futureReward = heuristicWeight *
                                                (double)
                                                remainingConsideredSteps() *
                                                initialQValue;
    node->children[actionIndex]->numberOfVisits = numberOfInitialVisits;

    node->numberOfVisits += numberOfInitialVisits;
    node->futureReward =
        std::max(node->futureReward,
                 node->children[actionIndex]->getExpectedRewardEstimate()); 

    //cout<<"CDP initializeDecisionNode  node->futureReward:"<<node->futureReward<< " node->expected"<<node->getExpectedRewardEstimate()<<endl;
    // cout << "initialized child ";
    // SearchEngine::actionStates[actionIndex].printCompact(cout);
    // cout << " with remaining steps " << remainingConsideredSteps() 
    //      << " and initialQValue " << initialQValue << endl;
    // node->children[actionIndex]->print(cout);
    // cout << endl;
}

/******************************************************************
                         Outcome selection
******************************************************************/

WMCUCTNode* WMCUCTSearch::selectOutcome(WMCUCTNode* node,
                                      PDState& nextState,
                                      int& varIndex) {
    DiscretePD& pd = nextState.probabilisticStateFluentAsPD(varIndex);
    assert(pd.isWellDefined());

    double probSum = 1.0;
    int childIndex = 0;

    if (node->children.empty()) {
        node->children.resize(
                SearchEngine::probabilisticCPFs[varIndex]->getDomainSize(),
                NULL);
    } else {
        // Determine the sum of the probabilities of unsolved outcomes
        for (unsigned int i = 0; i < pd.size(); ++i) {
            childIndex = pd.values[i];
            if (node->children[childIndex] &&
                node->children[childIndex]->isSolved()) {
                probSum -= pd.probabilities[i];
            }
        }
    }
    assert(MathUtils::doubleIsGreater(probSum, 0.0) && 
           MathUtils::doubleIsSmallerOrEqual(probSum, 1.0));

    double randNum = MathUtils::generateRandomNumber() * probSum;
    //cout << "ProbSum is " << probSum << endl;
    //cout << "RandNum is " << randNum << endl;

    probSum = 0.0;
    double childProb = 0.0;

    for (unsigned int i = 0; i < pd.size(); ++i) {
        childIndex = pd.values[i];
        if (!node->children[childIndex] ||
            !node->children[childIndex]->isSolved()) {
            probSum += pd.probabilities[i];
            if (MathUtils::doubleIsSmaller(randNum, probSum)) {
                childProb = pd.probabilities[i];
                break;
            }
        }
    }

    // cout << "Chosen child is " << childIndex << " and prob is " << childProb << endl;

    assert((childIndex >= 0) && childIndex < node->children.size());

    if (!node->children[childIndex]) {
        node->children[childIndex] = getWMCUCTNode(childProb);
    }

    assert(!node->children[childIndex]->isSolved());

    nextState.probabilisticStateFluent(varIndex) = childIndex;
    return node->children[childIndex];
}

/******************************************************************
                          Backup Functions
******************************************************************/

void WMCUCTSearch::backupDecisionNodeLeaf(WMCUCTNode* node,
                                         double const& immReward,
                                         double const& futReward) {
    node->children.clear();
    node->immediateReward = immReward;
    node->futureReward = futReward;
    node->firstReward = futReward;
    node->solved = true;
    ++node->numberOfVisits;
}

void WMCUCTSearch::backupDecisionNode(WMCUCTNode* node,
                                     double const& immReward,
                                     double const& /*futReward*/) {
    assert(!node->children.empty());
    node->immediateReward = immReward;
    if (selectedActionIndex() != -1) {
	//if (numeric_limits<double>::max() < node->firstReward)
	//	node->firstReward = futReward;

        ++node->numberOfVisits;
    }

    // Propagate values from best child
    //node->futureReward = numeric_limits<double>::max() < node->firstReward ? 0.0 : node->firstReward;
    //int visits = numeric_limits<double>::max() < node->firstReward ? 0 : 1;
    node->futureReward = 0.0;
    int visits = 0;
    node->solved = true;
    double maxValue = -numeric_limits<double>::max();
    for (unsigned int childIndex = 0; childIndex < node->children.size(); ++childIndex) {
        if (node->children[childIndex]) {
            node->solved &= node->children[childIndex]->solved;
            node->futureReward += node->children[childIndex]->getExpectedRewardEstimate() * (double) node->children[childIndex]->numberOfVisits;
	    visits += node->children[childIndex]->numberOfVisits;
	    maxValue = max(node->children[childIndex]->getExpectedRewardEstimate(), maxValue);
        }
    }

    if (node->solved)
	node->futureReward = maxValue;
    else
    	node->futureReward /= visits;
}

void WMCUCTSearch::backupChanceNode(WMCUCTNode* node,
                                   double const& /*futReward*/) {
    assert(MathUtils::doubleIsEqual(node->immediateReward, 0.0));

    ++node->numberOfVisits;

    // Propagate values from children
    node->futureReward = 0.0;
    double solvedSum = 0.0;
    double probSum = 0.0;

    for (unsigned int i = 0; i < node->children.size(); ++i) {
        if (node->children[i]) {
            node->futureReward += (node->children[i]->prob *
                                   node->children[i]->getExpectedRewardEstimate());
            probSum += node->children[i]->prob;

            if (node->children[i]->solved) {
                solvedSum += node->children[i]->prob;
            }
        }
    }

    node->futureReward /= probSum;
    node->solved = MathUtils::doubleIsEqual(solvedSum, 1.0);
}
