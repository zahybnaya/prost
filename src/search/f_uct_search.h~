#ifndef F_UCT_SEARCH_H
#define F_UCT_SEARCH_H

#include "uct_base.h"

class FUCTNode {
public:
    FUCTNode() :
        children(),
        immediateReward(0.0),
        futureReward(-std::numeric_limits<double>::max()),
	futureRewardSum(0.0),
        numberOfVisits(0),
        prob(0.0),
        solved(false),
        rewardLock(false) {}

    ~FUCTNode() {
        for (unsigned int i = 0; i < children.size(); ++i) {
            if (children[i]) {
                delete children[i];
            }
        }
    }

    friend class FUCTSearch;

    void reset() {
        children.clear();
        immediateReward = 0.0;
        futureReward = -std::numeric_limits<double>::max();
	futureRewardSum = 0.0;
        numberOfVisits = 0;
        prob = 0.0;
        solved = false;
        rewardLock = false;
    }

    double getExpectedRewardEstimate() const {
        return futureReward + immediateReward;
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

    std::vector<FUCTNode*> children;

private:
    double immediateReward;
    double futureReward;
    double futureRewardSum;
    int numberOfVisits;

    double prob;
    bool solved;
    bool rewardLock;
};

class FUCTSearch : public UCTBase<FUCTNode> {
public:
    FUCTSearch() :
        UCTBase<FUCTNode>("FUCT"),
        heuristicWeight(0.5) {}

    // Set parameters from command line
    bool setValueFromString(std::string& param, std::string& value) {
        if (param == "-hw") {
            setHeuristicWeight(atof(value.c_str()));
            return true;
        }

        return UCTBase<FUCTNode>::setValueFromString(param, value);
    }

    // Parameter setter
    virtual void setHeuristicWeight(double _heuristicWeight) {
        heuristicWeight = _heuristicWeight;
    }

protected:
    // Initialization of nodes
    void initializeDecisionNodeChild(FUCTNode* node,
            unsigned int const& actionIndex,
            double const& initialQValue);

    // Outcome selection
    FUCTNode* selectOutcome(FUCTNode* node, PDState& nextState, int& varIndex);

    // Backup functions
    void backupDecisionNodeLeaf(FUCTNode* node, double const& immReward, double const& futReward);
    void backupDecisionNode(FUCTNode* node, double const& immReward, double const& futReward);
    void backupChanceNode(FUCTNode* node, double const& futReward);

    // Memory management
    FUCTNode* getRootNode() {
        return getFUCTNode(1.0);
    }

    FUCTNode* getDummyNode() {
        return getFUCTNode(1.0);
    }

private:
    // Memory management
    FUCTNode* getFUCTNode(double const& prob) {
        FUCTNode* res = UCTBase<FUCTNode>::getSearchNode();
        res->prob = prob;
        return res;
    }

    // Parameter
    double heuristicWeight;
};

#endif
