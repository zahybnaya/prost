#include "cmc_uct_search.h"
#include "tValues.h"

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

//Preform a one-tail t-test to determine which node's mean is higher than the other.
//Actually called a Welch test for comparing means of unequal sample size and variance.
//Return 1 if node1's mean is significantly larger, 0 if there is no significant conclusion, or -1 if node2's mean is significantly larger.
int CMCUCTSearch::oneTailTTest(CMCUCTNode* node1, CMCUCTNode* node2) {
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
	t /= max(n1 + n2 - 2.0, 1.0);
	t *= ((1 / n1) + (1 / n2));
	t = delta / sqrt(t);

	double degree = (1.0 / max(n1 - 1.0, 1.0)) * pow(var1 / n1, 2);
	degree += (1.0 / max(n2 - 1.0, 1.0)) * pow(var2 / n2, 2);
	degree = pow((var1 / n1) + (var2 / n2), 2) / degree;

	int df = (int)round(degree);
	if (df > 999)
		df = 999;

	double criticalValue = t80[df];
	if (MathUtils::doubleIsGreater(t, criticalValue))
		return 1;
	else if (MathUtils::doubleIsSmaller(t, -1.0 * criticalValue))
		return -1;
	else
		return 0;
}

void CMCUCTSearch::backupDecisionNode(CMCUCTNode* node, double const& immReward, double const& /*futReward*/) {
    node->immediateReward += immReward;
    ++node->numberOfVisits;

    //initialize nPlus to all existing children, and figure out if this node is solved.
    //If solved, assign the maximal child
    CMCUCTNode **nPlus = new CMCUCTNode*[node->children.size()];
    for (unsigned int childIndex = 0; childIndex < node->children.size(); ++childIndex) {
	nPlus[childIndex] = 0; //initialization
    	if (node->children[childIndex]) {
		nPlus[childIndex] = node->children[childIndex];
        }
    }

    //Perform t-tests to decide which children should not be in nPlus
    for (unsigned int i = 0; i < node->children.size(); ++i) {
       //Check that a child exists, that it has more then one visits, and that it wasn't found as less significant before.
	 if (node->children[i] && nPlus[i]) {
	       for (unsigned int j = i + 1; j < node->children.size(); ++j) {
			//Check that that the child exists, that it has more than one visits, and that it wasn't found less significant before.
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
	if (nPlus[i]) {
		nPlusSum += (double)nPlus[i]->numberOfVisits * nPlus[i]->getExpectedRewardEstimate();
		nPlusVisits += nPlus[i]->numberOfVisits;
		nPlusCI += (double)(nPlus[i]->numberOfVisits - 1) * nPlus[i]->ci + (double)nPlus[i]->numberOfVisits * pow(nPlus[i]->getExpectedRewardEstimate(), 2);
	}
    }

    delete nPlus;

    node->futureReward = nPlusSum / (double)max(nPlusVisits, 1);

    nPlusCI = nPlusCI - (double)nPlusVisits * pow(node->futureReward, 2);
    nPlusCI = MathUtils::doubleIsEqual(nPlusCI, 0.0) ? 0.0 : nPlusCI;
    nPlusCI = (nPlusCI < 0.0) ? 0.0 : nPlusCI;
    node->ci = nPlusCI / (double)max(nPlusVisits - 1, 1);

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
    node->ci = (node->ci < 0) ? 0.0 : node->ci;
    node->ci = node->ci / (visits - 1.0); 
}
