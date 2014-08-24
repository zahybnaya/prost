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
    node->children[actionIndex]->expectedRewardEstimate = node->children[actionIndex]->futureReward;
    node->children[actionIndex]->numberOfVisits = numberOfInitialVisits;
 
    node->numberOfVisits += numberOfInitialVisits;
    node->futureReward =
        std::max(node->futureReward,
                 node->children[actionIndex]->getExpectedRewardEstimate()); //TODO this is done because in DP you take the max, so if the initial value is higher then take it... here in CDP it should be something else...

    node->expectedRewardEstimate = node->futureReward;
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
    //cout<<"CDP:	leaf. node->expectedReward" << node->getExpectedRewardEstimate()<<endl;
    // cout << "updated dec node leaf with immediate reward " << immReward << endl;
    // node->print(cout);
    // cout << endl;
}

void CDPUCTSearch::backupDecisionNode(CDPUCTNode* node,
                                     double const& immReward,
                                     double const& futReward) {
    assert(!node->children.empty());
    if (selectedActionIndex() != -1) {
        ++node->numberOfVisits;
    }

    if (backupLock) {
        ++skippedBackups;
        return;
    }

    node->immediateReward += immReward; 
    node->futureReward += futReward;
    double oldFutureReward = node->futureReward / double(node->numberOfVisits);
    //double oldImmediateReward = node->immediateReward / double(node->numberOfVisits);
    //node->expectedRewardEstimate = oldFutureReward + oldImmediateReward;
    node->expectedRewardEstimate = oldFutureReward; //TODO not sure if to add immidiate reward; currently trying to immitate DP-UCT
    node->solved = true;
    node->M2 += pow(((immReward + futReward) - node->expectedRewardEstimate), 2);
    node->ci = node->M2 / std::max(node->numberOfVisits - 1, 1);

    //cout<<"CDP: oldfuturereward: "<<oldFutureReward << " oldimmediatereward:" << oldImmediateReward <<" expectedreward:"<<node->expectedRewardEstimate<<" ci:"<<node->ci<<" M2:"<<node->M2<<endl;

   node->expectedRewardEstimate = -std::numeric_limits<double>::max() + oldImmediateReward;
    for (unsigned int childIndex = 0; childIndex < node->children.size(); ++childIndex) {
        if (node->children[childIndex]) {
            node->solved &= node->children[childIndex]->solved;
	  //  cout<<"child-ci: " <<node->children[childIndex]->ci << " child reward: "<<node->children[childIndex]->getExpectedRewardEstimate();
            if (//node->children[childIndex]->ci <= node->ci &&
                    node->children[childIndex]->getExpectedRewardEstimate() > node->getExpectedRewardEstimate() - oldImmediateReward) {
		    node->ci = node->children[childIndex]->getCi();
		    //node->expectedRewardEstimate = oldImmediateReward + node->children[childIndex]->getExpectedRewardEstimate();
		    node->expectedRewardEstimate = immReward + node->children[childIndex]->getExpectedRewardEstimate(); //TODO delete this line and uncomment the one above; this is just to try and imitate DP-UCT
            }
	    else{
	//	    cout<<endl;
	    }
        }
    }
    // If the future reward did not change we did not find a better node and
    // therefore do not need to update the rewards in preceding parents.
    if (!node->solved &&
        (remainingConsideredSteps() > maxLockDepth) &&
        MathUtils::doubleIsEqual(oldFutureReward, node->futureReward)) {
        backupLock = true;
    }

    //cout<<"CDP:backupDecisionNode expectedreward:"<<node->getExpectedRewardEstimate()<<endl;
    // cout << "updated dec node with immediate reward " << immReward << endl;
    // node->print(cout);
    // cout << endl;
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
    double solvedSum = 0.0;
    double probSum = 0.0;
    int validChildren = 0;

    for (unsigned int i = 0; i < node->children.size(); ++i) {
        if (node->children[i]) {
            node->futureReward += (node->children[i]->prob *
                                   node->children[i]->getExpectedRewardEstimate());
            probSum += node->children[i]->prob;
	    node->ci+= node->children[i]->ci; 
	    validChildren++;
            if (node->children[i]->solved) {
                solvedSum += node->children[i]->prob;
            }
        }
    }

    node->ci /= validChildren;
    node->futureReward /= probSum;
    node->expectedRewardEstimate = node->futureReward; 
    node->solved = MathUtils::doubleIsEqual(solvedSum, 1.0);
    //cout<<"CDP: backupChanceNode expectedreward:"<<node->getExpectedRewardEstimate()<<endl;
    // cout << "updated chance node:" << endl;
    // node->print(cout);
    // cout << endl;
}
