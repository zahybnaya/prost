#ifndef CMC_UCT_SEARCH_H
#define CMC_UCT_SEARCH_H

// CMCUCTSearch is the "standard" UCT variant that uses Monte-Carlo backups and
// Monte-Carlo sampling for outcome selection.

#include "uct_base.h"

class CMCUCTNode { //Monte Carlo UCTNode
public:
    CMCUCTNode() :
        children(),
        immediateReward(0.0),
        futureReward(-std::numeric_limits<double>::max()),
        numberOfVisits(0),
	ci(0.0),
        rewardLock(false) {}

    ~CMCUCTNode() {
        for (unsigned int i = 0; i < children.size(); ++i) {
            if (children[i]) {
                delete children[i];
            }
        }
    }

    friend class CMCUCTSearch;

    void reset() {
        children.clear();
        immediateReward = 0.0;
        futureReward = -std::numeric_limits<double>::max();
        numberOfVisits = 0;
	ci = 0.0;
        rewardLock = false;
    }

    double getExpectedRewardEstimate() const {
        return (immediateReward / (double) numberOfVisits) + futureReward;
    }

    double getExpectedFutureRewardEstimate() const {
        return futureReward;
    }

    bool isSolved() const {
        return false;
    }

    bool const& isARewardLock() const {
        return rewardLock;
    }

    void setRewardLock(bool const& _rewardLock) {
        rewardLock = _rewardLock;
    }

    int const& getNumberOfVisits() const {
        return numberOfVisits;
    }

    // Print
    void print(std::ostream& out, std::string indent = "") const {
        out << indent << getExpectedRewardEstimate() << " (in " <<
        numberOfVisits << " visits)" << std::endl;
    }

    std::vector<CMCUCTNode*> children;

    // Tests accessing private members of this class
    friend class UCTBaseTest;

private:
    double immediateReward;
    double futureReward;
    int numberOfVisits;
    double ci;

    bool rewardLock;
};

class CMCUCTSearch : public UCTBase<CMCUCTNode> {
public:
    CMCUCTSearch() :
        UCTBase<CMCUCTNode>("CMC-UCT"),
	tests(0),
	updates(0) {}

protected:
    // Initialization
    void initializeDecisionNodeChild(CMCUCTNode* node, unsigned int const& index,
            double const& initialQValue);

    // Outcome selection
    CMCUCTNode* selectOutcome(CMCUCTNode* node, PDState& nextState, int& varIndex);

    // Backup functions
    void backupDecisionNodeLeaf(CMCUCTNode* node, double const& immReward,
            double const& futReward) {
        // This is only different from backupDecisionNode if we want to label
        // nodes as solved, which is not possible in MonteCarlo-UCT.
        backupDecisionNode(node, immReward, futReward);
    }
    void backupDecisionNode(CMCUCTNode* node, double const& immReward,
            double const& futReward);
    void backupChanceNode(CMCUCTNode* node, double const& futReward);

private:
   int tests;
   int updates;    

    //Helper math functions
    int oneTailTTest(CMCUCTNode* node1, CMCUCTNode* node2);
};

#endif
