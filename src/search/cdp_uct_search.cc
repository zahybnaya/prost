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

void killNan(double d) {
	if (std::isnan(d)) {
		int a = 1;
		int b = 0;
		std::cout << a / b;
		cout << "killed nan" << endl;
	}
}

void killNegative(double d) {
	if (d < 0) {
		int a = 1;
		int b = 0;
		cout << "killed negative" << endl;
		std::cout << a / b;
	}
}

//Preform a one-tail t-test to determine wether node2's mean is higher than node1's.
//Actually called a Welch test for comparing means of unequal sample size and variance.
bool CDPUCTSearch::oneTailTTest(CDPUCTNode* node1, CDPUCTNode* node2) {
	double mean1 = node1->getExpectedRewardEstimate();
cout << endl << "mean 1 " << mean1 <<endl;
	double mean2 = node2->getExpectedRewardEstimate();
cout << "mean 2 " << mean2 <<endl;
	double var1 = node1->ci;
cout << "var 1 " << var1 <<endl;
	double var2 = node2->ci;
cout << "var 2 " << var2 <<endl;
	double n1 = (double)node1->numberOfVisits;
	double n2 = (double)node2->numberOfVisits;
killNan(mean1);
killNan(mean2);
killNan(var1);
killNan(var2);
killNan(n1);
killNan(n1);
	if (var1 == 0 && var2 == 0)
		return mean1 < mean2;

cout << "n 1 " << n1 <<endl;
cout << "n 2 " << n2 <<endl;
	double t = (n1 - 1.0) * var1 + (n2 - 1.0) * var2;
killNan(t);
	t /= n1 + n2 - 2.0;
killNan(t);
	t *= ((1 / n1) + (1 / n2));
killNan(t);
cout << "t before sqrt " << t <<endl;
	t = (mean1 - mean2) / sqrt(t);
killNan(t);
cout << "t after sqrt " << t <<endl;
	double degree = (1 / (n1 - 1)) * pow(var1 / n1, 2);
killNan(degree);
	degree += (1 / (n2 - 1)) * pow(var2 / n2, 2);
killNan(degree);
	degree = pow((var1 / n1) + (var2 / n2), 2) / degree;
killNan(degree);
cout << " degree " << degree << endl;
	int df = (int)round(degree);
	if (df > 999)
		df = 999;
cout<<"df " <<df <<endl;
cout<<"t < t_values[df] " <<(t < t_values[df]) <<endl;
	return t < t_values[df];
}

void CDPUCTSearch::backupDecisionNode(CDPUCTNode* node,
                                     double const& immReward,
                                     double const& /*futReward*/) {
    assert(!node->children.empty());
cout << " decision node " << endl;
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
killNan(weightedAvg);
		oldCI += (double)(childVisits - 1) * childCI + (double)childVisits * pow(childFutureRewards, 2); 
killNan(oldCI);
cout << "accumulating CI " << oldCI << endl;
killNegative(oldCI);
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
killNan(weightedAvg);
    oldCI = oldCI - (double)visits * pow(weightedAvg, 2);
cout << "oldCI after subtraction " << oldCI << endl;
killNan(oldCI);
//killNegative(oldCI); //We know this could possibly result in a negative
    oldCI = MathUtils::doubleIsEqual(oldCI, 0.0) ? 0.0 : oldCI;
    oldCI = (oldCI < 0) ? 0.0 : oldCI;
killNan(oldCI);
killNegative(oldCI);
    oldCI = oldCI / (double)max(visits - 1.0, 1.0);
killNegative(oldCI);
killNan(oldCI);
    //If the number of visits is under the threshold, stay with the wiegthed average.
    /*if (visits < node->children.size()) {
	node->futureReward = weightedAvg;
        node->ci = oldCI;
	return;
    }*/

    //Save the old reward for backup lock
    double oldFutureReward = node->futureReward;

    // Propagate values from best child
    CDPUCTNode **nPlus = new CDPUCTNode*[node->children.size()];
cout << " size of children vector " << node->children.size() << endl;
    for (unsigned int i = 0; i < node->children.size(); ++i) {
       nPlus[i] = 0; //initialization
       if (node->children[i] && node->children[i]->numberOfVisits > 1) {
cout << "child " << i;
	       nPlus[i] = node->children[i];
	       for (unsigned int j = 0; j < node->children.size(); ++j) {
			if (i != j && node->children[j] && node->children[j]->numberOfVisits > 1) {
cout << " compared with " << j;
			    if (oneTailTTest(node->children[i], node->children[j])) {
				nPlus[i] = 0;
			    }
			}
	       }
cout << endl;
       }
    }

    double nPlusSum = 0.0;
    double nPlusCI = 0.0;
    int nPlusVisits = 0;
    bool update = false;
    for (unsigned int i = 0; i < node->children.size(); ++i) {
cout << "checking nplus " << i << " " << nPlus[i] << endl;
	if (nPlus[i] != 0) {
		update = true;
cout<<"nPlus  " << i << " visits " << nPlus[i]->numberOfVisits << " value " << nPlus[i]->getExpectedRewardEstimate() << " ci " << nPlus[i]->ci<< endl;
		nPlusSum += (double)nPlus[i]->numberOfVisits * nPlus[i]->getExpectedRewardEstimate();
killNan(nPlusSum);
		nPlusVisits += nPlus[i]->numberOfVisits;
		nPlusCI += (double)(nPlus[i]->numberOfVisits - 1) * nPlus[i]->ci + (double)nPlus[i]->numberOfVisits * pow(nPlus[i]->getExpectedRewardEstimate(), 2); 

	cout<<"nPlus  " << i << " ci in the works " << nPlusCI << " avg in the works " << nPlusSum << " visits in the works " << nPlusVisits <<endl;
killNan(nPlusCI);
killNegative(nPlusCI);
	}
    }

    delete(nPlus);

    if (update) {
	node->futureReward = nPlusSum / (double)max(nPlusVisits, 1);
killNan(node->futureReward);
	nPlusCI = nPlusCI - (double)nPlusVisits * pow(node->futureReward, 2);
cout << "nPlusCI after subtraction " << nPlusCI << endl;
killNan(nPlusCI);
//killNegative(nPlusCI); //We know this could possibly be negative
	nPlusCI = MathUtils::doubleIsEqual(nPlusCI, 0.0) ? 0.0 : nPlusCI;
	nPlusCI = (nPlusCI < 0.0) ? 0.0 : nPlusCI;
cout << "nPlusCI after rounding " << nPlusCI << endl;
killNan(nPlusCI);
killNegative(nPlusCI);
	node->ci = nPlusCI / (double)max(nPlusVisits - 1, 1);
cout << "nPlusCI " << node->ci << " nPlus avg " << node->futureReward << endl<< endl;
killNan(node->ci);
killNegative(node->ci);
    } else {
	node->futureReward = weightedAvg;
	node->ci = oldCI;
cout << "parent ci " << node->ci << " parent value " << node->futureReward << endl<< endl;
    }

//if (std::isnan(node->ci)) {
/*if (node->numberOfVisits > 20) {
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
cout << " chance node " << endl;
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
cout << "ci in the works " << node->ci << endl;
killNegative(node->ci);

            probSum += childProb;

            if (node->children[i]->solved)
                solvedSum += childProb;
        }
    }

    node->futureReward /= visits;
cout << "node->ci " << node->ci << " visits * pow(node->futureReward, 2) "  << (visits * pow(node->futureReward, 2)) << endl;
    node->ci = node->ci - visits * pow(node->futureReward, 2);
cout << "ci after subtraction " << node->ci << endl;
//killNegative(node->ci); //We know this could possibly be negative
    node->ci = MathUtils::doubleIsEqual(node->ci, 0.0) ? 0.0 : node->ci;
    node->ci = (node->ci < 0) ? 0.0 : node->ci;
cout << "ci after rounding fix " << node->ci << endl;
killNegative(node->ci);
    node->ci = node->ci / (visits - 1.0); 
killNegative(node->ci);
    node->solved = MathUtils::doubleIsEqual(solvedSum, 1.0);
}
