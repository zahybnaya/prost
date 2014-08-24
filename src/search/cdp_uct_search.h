#ifndef CDP_UCT_SEARCH_H
#define CDP_UCT_SEARCH_H

// CDPUCTSearch is used for two of the UCT variants described in the
// ICAPS 2013 paper by Keller and Helmert. If called with IDS as
// initializers, it corresponds to the search engine labelled CDP-UCT
// in that paper which uses Partial Bellman backups and Monte-Carlo
// sampling for outcome selection. If the number of new decision nodes
// is limited to 1, it is UCT*.

#include "uct_base.h"

class CDPUCTNode {
public:
    CDPUCTNode() :
        children(),
        immediateReward(0.0),
	M2(0.0),
        futureReward(-std::numeric_limits<double>::max()),
	ci(std::numeric_limits<double>::max()),
	expectedRewardEstimate(-std::numeric_limits<double>::max()),
        numberOfVisits(0),
        prob(0.0),
        solved(false),
        rewardLock(false) {}

    ~CDPUCTNode() {
        for (unsigned int i = 0; i < children.size(); ++i) {
            if (children[i]) {
                delete children[i];
            }
        }
    }

    friend class CDPUCTSearch;

    void reset() {
        children.clear();
        immediateReward = 0.0;
        futureReward = -std::numeric_limits<double>::max();
        numberOfVisits = 0;
        prob = 0.0;
        M2 = 0.0;
	expectedRewardEstimate = -std::numeric_limits<double>::max();
        solved = false;
        rewardLock = false;
	ci = std::numeric_limits<double>::max();
    }

    double getExpectedRewardEstimate() const {
        return expectedRewardEstimate;
    }

    double getCi() {
	return ci;
    }

    double getExpectedFutureRewardEstimate() const {
        return futureReward;
    }

    bool const& isSolved() const {
        return solved;
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

    void print(std::ostream& out, std::string indent = "") const {
        if (solved) {
            out << indent << "SOLVED with: " << getExpectedRewardEstimate() <<
            " (in " << numberOfVisits << " real visits)" << std::endl;
        } else {
            out << indent << getExpectedRewardEstimate() << " (in " <<
            numberOfVisits << " real visits)" << std::endl;
        }
    }

    std::vector<CDPUCTNode*> children;

private:
    double immediateReward;
    double M2;
    double futureReward;
    double ci;
    double expectedRewardEstimate;
    int numberOfVisits;

    double prob;
    bool solved;
    bool rewardLock;
};

class CDPUCTSearch : public UCTBase<CDPUCTNode> {
public:
    CDPUCTSearch() :
        UCTBase<CDPUCTNode>("CDP-UCT"),
        heuristicWeight(0.5) {}

    // Set parameters from command line
    bool setValueFromString(std::string& param, std::string& value) {
        if (param == "-hw") {
            setHeuristicWeight(atof(value.c_str()));
            return true;
        }

        return UCTBase<CDPUCTNode>::setValueFromString(param, value);
    }

    // Parameter setter
    virtual void setHeuristicWeight(double _heuristicWeight) {
        heuristicWeight = _heuristicWeight;
    }

protected:
    // Initialization of nodes
    void initializeDecisionNodeChild(CDPUCTNode* node,
            unsigned int const& actionIndex,
            double const& initialQValue);

    // Outcome selection
    CDPUCTNode* selectOutcome(CDPUCTNode* node, PDState& nextState, int& varIndex);

    // Backup functions
    void backupDecisionNodeLeaf(CDPUCTNode* node, double const& immReward,
            double const& futReward);
    void backupDecisionNode(CDPUCTNode* node, double const& immReward,
            double const& futReward);
    void backupChanceNode(CDPUCTNode* node, double const& futReward);

    // Memory management
    CDPUCTNode* getRootNode() {
        return getCDPUCTNode(1.0);
    }

    CDPUCTNode* getDummyNode() {
        return getCDPUCTNode(1.0);
    }

private:
    // Memory management
    CDPUCTNode* getCDPUCTNode(double const& prob) {
        CDPUCTNode* res = UCTBase<CDPUCTNode>::getSearchNode();
        res->prob = prob;
        return res;
    }

    // Parameter
    double heuristicWeight;
};

#endif
