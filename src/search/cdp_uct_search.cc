#include "cdp_uct_search.h"

using namespace std;

/******************************************************************
                     Initialization of Nodes
*******************************************************************/

void CDPUCTSearch::initializeDecisionNodeChild(CDPUCTNode* node,
                                              unsigned int const& actionIndex,
                                              double const& initialQValue) {
    node->children[actionIndex] = getCDPUCTNode(1.0);
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
                                     double const& /*futReward*/) {
    assert(!node->children.empty());

    node->immediateReward = immReward;
    if (selectedActionIndex() != -1) {
	    ++node->numberOfVisits;
    }
    if (backupLock) {
        ++skippedBackups;
        return;
    }

    //Calculate the weighted average and confidence of the childrens rewards.
    //Also check if this node is solved, and if so assign it the maximum expected child reward.
    double weightedAvg = 0.0;
    double oldCI = 0.0;
    double maxValue = -numeric_limits<double>::max();
    int visits = 0;
    node->solved = true;
    for (unsigned int childIndex = 0; childIndex < node->children.size(); ++childIndex) {
    	if (node->children[childIndex]) {
		node->solved &= node->children[childIndex]->solved;
		weightedAvg += node->children[childIndex]->getExpectedRewardEstimate() * (double) node->children[childIndex]->numberOfVisits;
		oldCI += ((double)pow(node->children[childIndex]->numberOfVisits, 2) * node->children[childIndex]->ci); 
		maxValue = max(node->children[childIndex]->getExpectedRewardEstimate(), maxValue);
		visits += node->children[childIndex]->numberOfVisits;

		//std::cout << "ci " << node->children[childIndex]->ci << " visits " << node->children[childIndex]->numberOfVisits << std::endl;

        }
    }

    if (node->solved) {
	node->futureReward = maxValue;
	node->ci = 0;
	return;
    }

    weightedAvg /= (double)max(visits, 1);
    oldCI /= (double)pow(max(visits, 1), 2);

    //Save the old reward for backup lock
    double oldFutureReward = node->futureReward;

    // Propagate values from best child
    node->futureReward = weightedAvg;
    node->ci = oldCI;
   // bool updated = false;
//std::cout << weightedAvg << " wAVG " << oldCI << " CI " << node->children.size() << " num children" << std::endl;
    for (unsigned int childIndex = 0; childIndex < node->children.size(); ++childIndex) {
        if (node->children[childIndex]) {

	    if (MathUtils::doubleIsGreater(node->children[childIndex]->getExpectedRewardEstimate(), node->futureReward) &&
		MathUtils::doubleIsSmaller(node->children[childIndex]->ci, oldCI)) {

		node->futureReward = node->children[childIndex]->getExpectedRewardEstimate();
		node->ci = node->children[childIndex]->ci;
		//updated = true;
	    }
        }
    }
    
    /*tests++;
    if (updated) {
	updates++;
	//std::cout << "first updated" << std::endl;
} else {
	//std::cout << "first not updated" << std::endl;
}

int foo = 1;
int bar = 0;
if (tests == 2)
std::cout << foo / bar;*/

    // If the future reward did not change we did not find a better node and
    // therefore do not need to update the rewards in preceding parents.
    if ((remainingConsideredSteps() > maxLockDepth) &&
        MathUtils::doubleIsEqual(oldFutureReward, node->futureReward)) {
        backupLock = true;
    }
}

void CDPUCTSearch::backupChanceNode(CDPUCTNode* node,
                                   double const& /*futReward*/) {
    assert(MathUtils::doubleIsEqual(node->immediateReward, 0.0));

    ++node->numberOfVisits;
    if (backupLock) {
        ++skippedBackups;
        return;
    }

    // Propagate values from children
    node->futureReward = 0.0;
    node->ci = 0.0;
    double solvedSum = 0.0;
    double probSum = 0.0;

    for (unsigned int i = 0; i < node->children.size(); ++i) {
        if (node->children[i]) {
            node->futureReward += (node->children[i]->prob *
                                   node->children[i]->getExpectedRewardEstimate());
            probSum += node->children[i]->prob;
	    node->ci += ((double)pow(node->children[i]->prob, 2) * node->children[i]->ci); 

            if (node->children[i]->solved) {
                solvedSum += node->children[i]->prob;
            }
        }
    }

    node->ci /= (double)pow(probSum, 2);
    node->futureReward /= probSum;
    node->solved = MathUtils::doubleIsEqual(solvedSum, 1.0);
}
