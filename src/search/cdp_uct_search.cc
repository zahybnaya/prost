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

//Preform a one-tail t-test to determine which node's mean is higher than the other.
//Actually called a Welch test for comparing means of unequal sample size and variance.
//Return 1 if node1's mean is significantly larger, 0 if there is no significant conclusion, or -1 if node2's mean is significantly larger.
int CDPUCTSearch::oneTailTTest(CDPUCTNode* node1, CDPUCTNode* node2) {
	double mean1 = node1->getExpectedRewardEstimate();
	double mean2 = node2->getExpectedRewardEstimate();

	double var1 = node1->ci;
	double var2 = node2->ci;

	double n1 = (double)node1->numberOfVisits;
	double n2 = (double)node2->numberOfVisits;

	double delta = mean1 - mean2;
	delta = MathUtils::doubleIsEqual(delta, 0.0) ? 0.0 : delta;

	if (var1 == 0 && var2 == 0) {
		if (MathUtils::doubleIsGreater(delta, 0.0))
			return 1;
		else if (MathUtils::doubleIsSmaller(delta, 0.0))
			return -1;
		else
			return 0;
	}

	double t = max(n1 - 1.0, 1.0) * var1 + max(n2 - 1.0, 1.0) * var2;
//	double t = (n1 - 1.0) * var1 + (n2 - 1.0) * var2; //this goes when not comparing samples of size one
	t /= max(n1 + n2 - 2.0, 1.0);
//	t /= n1 + n2 - 2.0; //this goes when not comparing samples of size one
	t *= ((1 / n1) + (1 / n2));

	t = delta / sqrt(t);
	double degree = (1.0 / max(n1 - 1.0, 1.0)) * pow(var1 / n1, 2);
//	double degree = (1 / (n1 - 1)) * pow(var1 / n1, 2); //this goes when not comparing samples of size one
	degree += (1.0 / max(n2 - 1.0, 1.0)) * pow(var2 / n2, 2);
//	degree += (1 / (n2 - 1)) * pow(var2 / n2, 2);  //this goes when not comparing samples of size one
	degree = pow((var1 / n1) + (var2 / n2), 2) / degree;

	int df = (int)round(degree);
	if (df > 999)
		df = 999;

	//double criticalValue = t95[df];
	double criticalValue = t70[df];
//criticalValue = 0; //for sanity check - with this CDP should converge with UCTStar
	if (MathUtils::doubleIsGreater(t, criticalValue))
		return 1;
	else if (MathUtils::doubleIsSmaller(t, -1.0 * criticalValue))
		return -1;
	else
		return 0;
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

    //initialize nPlus to all existing children, and figure out if this node is solved.
    //If solved, assign the maximal child
    CDPUCTNode **nPlus = new CDPUCTNode*[node->children.size()];
    double maxValue = -numeric_limits<double>::max();
    node->solved = true;
    for (unsigned int childIndex = 0; childIndex < node->children.size(); ++childIndex) {
	nPlus[childIndex] = 0; //initialization
    	if (node->children[childIndex]) {
		nPlus[childIndex] = node->children[childIndex];
		node->solved &= node->children[childIndex]->solved;
		maxValue = max(node->children[childIndex]->getExpectedRewardEstimate(), maxValue);
        }
    }

    if (node->solved) {
	node->futureReward = maxValue;
	node->ci = 0;
	return;
    }

    //Save the old reward for backup lock
    double oldFutureReward = node->futureReward;

    //Perform t-tests to decide which children should not be in nPlus
    for (unsigned int i = 0; i < node->children.size(); ++i) {
       //Check that a child exists, that it has more then one visits, and that it wasn't found as less significant before.
       //if (node->children[i] && node->children[i]->numberOfVisits > 1 && nPlus[i]) { //this goes when not comparing samples of size one
	 if (node->children[i] && nPlus[i]) {
	       for (unsigned int j = i + 1; j < node->children.size(); ++j) {
			//Check that that the child exists, that it has more than one visits, and that it wasn't found less significant before.
			//if (node->children[j] && node->children[j]->numberOfVisits > 1 && nPlus[j]) { //this goes when not comparing samples of size one
			if (node->children[j] && nPlus[j]) {
			    int res = oneTailTTest(node->children[i], node->children[j]);
			    if (res == 1) {
				nPlus[j] = 0;
			    } else if (res == -1) {
				nPlus[i] = 0;
				break;
			    }
			}
	       }
       }
    }

    double nPlusSum = 0.0;
    double nPlusCI = 0.0;
    int nPlusVisits = 0;
    for (unsigned int i = 0; i < node->children.size(); ++i) {
//if (debugMessage) {  cout << "checking nplus " << i << " " << nPlus[i] << endl;}
	if (nPlus[i]) {
//if (debugMessage) { cout<<"nPlus  " << i << " visits " << nPlus[i]->numberOfVisits << " value " << nPlus[i]->getExpectedRewardEstimate() << " ci " << nPlus[i]->ci<< endl; }
		nPlusSum += (double)nPlus[i]->numberOfVisits * nPlus[i]->getExpectedRewardEstimate();
//if (debugMessage) {  killNan(nPlusSum); }
		nPlusVisits += nPlus[i]->numberOfVisits;
		nPlusCI += (double)(nPlus[i]->numberOfVisits - 1) * nPlus[i]->ci + (double)nPlus[i]->numberOfVisits * pow(nPlus[i]->getExpectedRewardEstimate(), 2); 

/*if (debugMessage) { 
	cout<<"nPlus  " << i << " ci in the works " << nPlusCI << " avg in the works " << nPlusSum << " visits in the works " << nPlusVisits <<endl;
killNan(nPlusCI);
killNegative(nPlusCI);
}*/
	}
    }

    delete nPlus;

    node->futureReward = nPlusSum / (double)max(nPlusVisits, 1);
//if (debugMessage) {  killNan(node->futureReward); }
    nPlusCI = nPlusCI - (double)nPlusVisits * pow(node->futureReward, 2);
	
/*if (debugMessage) { 
cout << "nPlusCI after subtraction " << nPlusCI << endl;
killNan(nPlusCI);
//killNegative(nPlusCI); //We know this could possibly be negative
}*/
	nPlusCI = MathUtils::doubleIsEqual(nPlusCI, 0.0) ? 0.0 : nPlusCI;
	nPlusCI = (nPlusCI < 0.0) ? 0.0 : nPlusCI;
/*if (debugMessage) { 
cout << "nPlusCI after rounding " << nPlusCI << endl;
killNan(nPlusCI);
killNegative(nPlusCI);
}*/
	node->ci = nPlusCI / (double)max(nPlusVisits - 1, 1);
/*if (debugMessage) { 
cout << "nPlusCI " << node->ci << " nPlus avg " << node->futureReward << endl<< endl;
killNan(node->ci);
killNegative(node->ci);
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
//if (debugMessage) {  cout << " chance node " << endl; }
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
/*if (debugMessage) { 
cout << "ci in the works " << node->ci << endl;
killNegative(node->ci);
}*/

            probSum += childProb;

            if (node->children[i]->solved)
                solvedSum += childProb;
        }
    }

    node->futureReward /= visits;
/*if (debugMessage) { 
cout << "node->ci " << node->ci << " visits * pow(node->futureReward, 2) "  << (visits * pow(node->futureReward, 2)) << endl;
}*/
    node->ci = node->ci - visits * pow(node->futureReward, 2);
/*if (debugMessage) { 
cout << "ci after subtraction " << node->ci << endl;
//killNegative(node->ci); //We know this could possibly be negative
}*/
    node->ci = MathUtils::doubleIsEqual(node->ci, 0.0) ? 0.0 : node->ci;
    node->ci = (node->ci < 0) ? 0.0 : node->ci;
/*if (debugMessage) { 
cout << "ci after rounding fix " << node->ci << endl;
killNegative(node->ci);
}*/
    node->ci = node->ci / (visits - 1.0); 
//if (debugMessage) { killNegative(node->ci); }
    node->solved = MathUtils::doubleIsEqual(solvedSum, 1.0);
}
