#include "cdp_uct_search.h"
#include "tValues.h"

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

bool isInfinity(double n) {
	return numeric_limits<double>::max() < n;
}

//Preform a one-tail t-test to determine wether node2's mean is higher than node1's.
bool CDPUCTSearch::oneTailTTest(CDPUCTNode* node1, CDPUCTNode* node2) {
	double mean1 = node1->getExpectedRewardEstimate();
	double mean2 = node2->getExpectedRewardEstimate();

	double var1 = pow(node1->ci, 2);
	double var2 = pow(node2->ci, 2);

	double n1 = (double)node1->numberOfVisits;
	double n2 = (double)node2->numberOfVisits;

	double t = (n1 - 1.0) * var1 + (n2 - 1.0) * var2;
	t /= n1 + n2 - 2.0;
	t *= ((1 / n1) + (1 / n2));
	t = (mean1 - mean2) / sqrt(t);

	double degree = (1 / (n1 - 1)) * pow(var1 / n1, 2);
	degree += (1 / (n2 - 1)) * pow(var2 / n2, 2);
	degree = pow((var1 / n1) + (var2 / n2), 2) / degree;

	int df = (int)round(degree);
	if (df > 999)
		df = 999;
	
	return t < t_values[df];
}

void CDPUCTSearch::backupDecisionNode(CDPUCTNode* node,
                                     double const& immReward,
                                     double const& /*futReward*/) {
    assert(!node->children.empty());

    node->immediateReward = immReward;
    if (selectedActionIndex() != -1) {
	//if (isInfinity(node->firstReward))
	//	node->firstReward = futReward;

	++node->numberOfVisits;
    }
    if (backupLock) {
        ++skippedBackups;
        return;
    }

    //Calculate the weighted average and confidence of the childrens rewards.
    //Also check if this node is solved, and if so assign it the maximum expected child reward.
    double weightedAvg = 0.0;
    int visits = 0;
    double oldCI = 0.0;

    //If there is a first reward, include it.
    /*if (!isInfinity(node->firstReward)) {
	weightedAvg = node->firstReward;
	visits = 1;
	oldCI = pow(node->firstReward, 2);
    }*/

    double maxValue = -numeric_limits<double>::max();
    node->solved = true;
    for (unsigned int childIndex = 0; childIndex < node->children.size(); ++childIndex) {
    	if (node->children[childIndex]) {
		int childVisits = node->children[childIndex]->numberOfVisits;
		double childFutureRewards = node->children[childIndex]->getExpectedRewardEstimate();
		double childCI = node->children[childIndex]->ci;

		node->solved &= node->children[childIndex]->solved;
		weightedAvg += childFutureRewards * (double)childVisits;
		oldCI += (double)(childVisits - 1) * childCI + (double)childVisits * pow(childFutureRewards, 2); 
		maxValue = max(childFutureRewards, maxValue);
		visits += childVisits;
        }
    }

    if (node->solved) {
	node->futureReward = maxValue;
	node->ci = 0;
	return;
    }

    weightedAvg /= (double)max(visits, 1);
    oldCI = oldCI - (double)visits * pow(weightedAvg, 2);
    oldCI = MathUtils::doubleIsEqual(oldCI, 0.0) ? 0.0 : oldCI;
    oldCI = (oldCI < 0) ? 0.0 : oldCI;
    oldCI = oldCI / max(visits - 1.0, 1.0);

    //If the number of visits is under the threshold, stay with the wiegthed average.
    /*if (visits < node->children.size()) {
	node->futureReward = weightedAvg;
        node->ci = oldCI;
	return;
    }*/

    //Save the old reward for backup lock
    double oldFutureReward = node->futureReward;
/*
    //Keep for reassigning after all tests are done.
    //T-test requiers that the number of visits match the sampled populations.
    int oldVisits = node->numberOfVisits;
*/
    // Propagate values from best child
    node->futureReward = weightedAvg;
    node->ci = oldCI;
    bool updated = false;
    CDPUCTNode* nPlus[node->children.size()];
    for (unsigned int childIndex = 0; childIndex < node->children.size(); ++childIndex) {
        if (node->children[childIndex]) {

	    if (MathUtils::doubleIsGreater(node->children[childIndex]->getExpectedRewardEstimate(), node->futureReward) &&
		MathUtils::doubleIsSmaller(node->children[childIndex]->ci, oldCI)) {
	    //if (oneTailTTest(node, node->children[childIndex])) {

		node->futureReward = node->children[childIndex]->getExpectedRewardEstimate();
		node->ci = node->children[childIndex]->ci;
		//node->numberOfVisits = node->children[childIndex]->numberOfVisits; for t test
		updated = true;
	    }
        }
    }

//    node->numberOfVisits = oldVisits; for t test
    
    tests++;
    if (updated)
	updates++;
/*
if (std::isnan(node->ci)) {
double vis = 6000000;
double val = -15.5833333333333;
double sum = 0.0;
std::cout << "vis " << vis << " val " << val << " sum " << sum << std::endl;
sum += vis * val;
std::cout << "this is just a tribute " <<  sum << std::endl;
int a = 1;
int b = 0;
std::cout << a / b;
}*/
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
    double visits = 0.0;
    for (unsigned int i = 0; i < node->children.size(); ++i) {
        if (node->children[i]) {
	    double childCI = node->children[i]->ci;
	    double childProb = node->children[i]->prob;
	    double childFutureRewards = node->children[i]->getExpectedRewardEstimate();
	    double childVisits = (double)round(100000.0 * childProb);

	    visits += childVisits;
            node->futureReward += childVisits * childFutureRewards;
	    node->ci += (childVisits - 1) * childCI + childVisits * pow(childFutureRewards, 2);
            probSum += childProb;

            if (node->children[i]->solved)
                solvedSum += childProb;
        }
    }

    node->futureReward /= visits;
    node->ci = node->ci - visits * pow(node->futureReward, 2);
    node->ci = MathUtils::doubleIsEqual(node->ci, 0.0) ? 0.0 : node->ci;
    node->ci = (node->ci < 0) ? 0.0 : node->ci;
    node->ci = node->ci / (visits - 1.0); 
    node->solved = MathUtils::doubleIsEqual(solvedSum, 1.0);
}
