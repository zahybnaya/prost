#include "f_uct_search.h"

using namespace std;

/******************************************************************
                     Initialization of Nodes
******************************************************************/

void FUCTSearch::initializeDecisionNodeChild(FUCTNode* node,
                                              unsigned int const& actionIndex,
                                              double const& initialQValue) {
    node->children[actionIndex] = getFUCTNode(1.0);
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

FUCTNode* FUCTSearch::selectOutcome(FUCTNode* node,
                                      PDState& nextState,
                                      int& varIndex) {
    DiscretePD& pd = nextState.probabilisticStateFluentAsPD(varIndex);
    assert(pd.isWellDefined());

    double probSum = 1.0;
    int childIndex = 0;

    if (node->children.empty()) {
        node->children.resize(SearchEngine::probabilisticCPFs[varIndex]->getDomainSize(), NULL);
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
        node->children[childIndex] = getFUCTNode(childProb);
    }

    assert(!node->children[childIndex]->isSolved());

    nextState.probabilisticStateFluent(varIndex) = childIndex;
    return node->children[childIndex];
}

/******************************************************************
                          Backup Functions
******************************************************************/

void FUCTSearch::backupDecisionNodeLeaf(FUCTNode* node,
                                         double const& immReward,
                                         double const& futReward) {
    node->children.clear();
    node->immediateReward = immReward;
    node->futureReward = futReward;
    node->solved = true;
    ++node->numberOfVisits;
}

void FUCTSearch::backupDecisionNode(FUCTNode* node,
                                     double const& immReward,
                                     double const& futReward) {
    assert(!node->children.empty());

    node->immediateReward = immReward;
    if (selectedActionIndex() != -1) {
	    node->numberOfVisits++;
	    node->futureRewardSum += futReward;
    }

    int maxValueIndex = -1;
    double maxValue = -std::numeric_limits<double>::max();
    int maxVisitsIndex = -1;
    int maxVisits = 0;
    double wAVG = 0.0;
    int visits = 0;
    node->solved = true;
    for (unsigned int childIndex = 0; childIndex < node->children.size(); ++childIndex) {
    	if (node->children[childIndex]) {
		node->solved &= node->children[childIndex]->solved;
		wAVG += (double)node->children[childIndex]->numberOfVisits * node->children[childIndex]->getExpectedRewardEstimate();
		visits += node->children[childIndex]->numberOfVisits;
		if (node->children[childIndex]->getExpectedRewardEstimate() >= maxValue) {
			maxValue = node->children[childIndex]->getExpectedRewardEstimate();
			maxValueIndex = childIndex;
		}
		if (node->children[childIndex]->numberOfVisits >= maxVisits) {
			maxVisits = node->children[childIndex]->numberOfVisits;
			maxVisitsIndex = childIndex;
		}
        }
    }

    wAVG /= (double)visits;
    assert(maxValueIndex != -1 && maxVisitsIndex != -1);
    if (maxValueIndex == maxVisitsIndex || node->solved)
	node->futureReward = maxValue;
    else
	node->futureReward = wAVG;
	//node->futureReward = node->futureRewardSum / (double)node->numberOfVisits;

//std::cout << "blah blah blah " << node->futureReward << std::endl;

}

void FUCTSearch::backupChanceNode(FUCTNode* node,
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
