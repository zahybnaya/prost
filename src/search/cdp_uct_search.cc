#include "cdp_uct_search.h"

using namespace std;

/******************************************************************
                     Initialization of Nodes
******************************************************************/

void CDPUCTSearch::initializeDecisionNodeChild(CDPUCTNode* node,
                                              unsigned int const& actionIndex,
                                              double const& initialQValue) {
    node->children[actionIndex] = getCDPUCTNode(1.0);
    node->children[actionIndex]->futureReward = heuristicWeight *
                                                (double)
                                                remainingConsideredSteps() *
                                                initialQValue;
    node->children[actionIndex]->numberOfVisits = numberOfInitialVisits;
//node->children[actionIndex]->ci = node->ci;

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

CDPUCTNode* CDPUCTSearch::selectOutcome(CDPUCTNode* node,
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
        node->children[childIndex] = getCDPUCTNode(childProb);
    }

    assert(!node->children[childIndex]->isSolved());

    nextState.probabilisticStateFluent(varIndex) = childIndex;
    return node->children[childIndex];
}

/******************************************************************
                          Backup Functions
******************************************************************/

void CDPUCTSearch::backupDecisionNodeLeaf(CDPUCTNode* node,
                                         double const& immReward,
                                         double const& futReward) {
    node->children.clear();
    node->immediateReward = immReward;
    node->futureReward = futReward;
    node->solved = true;
    node->ci = 0;
    ++node->numberOfVisits;
}

void CDPUCTSearch::backupDecisionNode(CDPUCTNode* node,
                                     double const& immReward,
                                     double const& futReward) {
    assert(!node->children.empty());

    node->immediateReward = immReward;
    if (selectedActionIndex() != -1) {
	    int n = node->numberOfVisits;
	    double mean = node->futureRewardSum / (double) n;
	    double delta = futReward - mean;
	    n++;
	    mean = mean + delta / (double)n;
	    node->M2 += delta * (futReward - mean);
	    node->numberOfVisits = n;
	    node->futureRewardSum += futReward;
    }

    double weightedAvg = 0.0;
    double oldCI = 0.0;
    for (unsigned int childIndex = 0; childIndex < node->children.size(); ++childIndex) {
    	if (node->children[childIndex]) {
		weightedAvg += node->children[childIndex]->getExpectedRewardEstimate() * (double) node->children[childIndex]->numberOfVisits;
		oldCI += ((double)pow(node->children[childIndex]->numberOfVisits, 2) * node->children[childIndex]->ci); 
        }
    }
    weightedAvg /= (double)node->numberOfVisits;
    oldCI /= (double)pow(max(node->numberOfVisits - 1, 1), 2);

    // Propagate values from best child
    node->futureReward = weightedAvg;
    node->ci = oldCI;
    node->solved = true;
    for (unsigned int childIndex = 0; childIndex < node->children.size(); ++childIndex) {
        if (node->children[childIndex]) {
            node->solved &= node->children[childIndex]->solved;
	    
   	    if (MathUtils::doubleIsSmaller(node->children[childIndex]->ci, oldCI) &&
		MathUtils::doubleIsGreater(node->children[childIndex]->getExpectedRewardEstimate(), node->futureReward)) {

		node->futureReward = node->children[childIndex]->getExpectedRewardEstimate();
		node->ci = node->children[childIndex]->ci;
	    }   else {
//cout << "old CI " << oldCI << " child ci " << node->children[childIndex]->ci << " future reward " << node->futureReward << " child future reward " << node->children[childIndex]->getExpectedRewardEstimate() << endl;
} 
        }
    }
}

void CDPUCTSearch::backupChanceNode(CDPUCTNode* node,
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
	    node->ci += (pow(node->children[i]->prob, 2) * node->children[i]->ci); 

            if (node->children[i]->solved) {
                solvedSum += node->children[i]->prob;
            }
        }
    }

    node->futureReward /= probSum;
    node->solved = MathUtils::doubleIsEqual(solvedSum, 1.0);
}
