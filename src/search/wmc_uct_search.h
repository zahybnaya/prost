#ifndef WMC_UCT_SEARCH_H
#define WMC_UCT_SEARCH_H

// WMCUCTSearch is used for two of the UCT variants described in the
// ICAPS 2013 paper by Keller and Helmert. If called with IDS as
// initializers, it corresponds to the search engine labelled CDP-UCT
// in that paper which uses Partial Bellman backups and Monte-Carlo
// sampling for outcome selection. If the number of new decision nodes
// is limited to 1, it is UCT*.

#include "uct_base.h"

class WMCUCTNode {
public:
    WMCUCTNode() :
        children(),
        immediateReward(0.0),
        futureReward(-std::numeric_limits<double>::max()),
	//firstReward(std::numeric_limits<double>::infinity()),
	numberOfVisits(0),
        prob(0.0),
        solved(false),
        rewardLock(false) {}

    ~WMCUCTNode() {
        for (unsigned int i = 0; i < children.size(); ++i) {
            if (children[i]) {
                delete children[i];
            }
        }
    }

    friend class WMCUCTSearch;

    void reset() {
        children.clear();
        immediateReward = 0.0;
        futureReward = -std::numeric_limits<double>::max();
	//firstReward = std::numeric_limits<double>::infinity();
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

    std::vector<WMCUCTNode*> children;

private:
    double immediateReward;
    double futureReward;
    double firstReward;
    int numberOfVisits;

    double prob;
    bool solved;
    bool rewardLock;
};

class WMCUCTSearch : public UCTBase<WMCUCTNode> {
public:
    WMCUCTSearch() :
        UCTBase<WMCUCTNode>("WMC-UCT"),
        heuristicWeight(0.5) {}

    // Set parameters from command line
    bool setValueFromString(std::string& param, std::string& value) {
        if (param == "-hw") {
            setHeuristicWeight(atof(value.c_str()));
            return true;
        }

        return UCTBase<WMCUCTNode>::setValueFromString(param, value);
    }

    // Parameter setter
    virtual void setHeuristicWeight(double _heuristicWeight) {
        heuristicWeight = _heuristicWeight;
    }

protected:
    // Initialization of nodes
    void initializeDecisionNodeChild(WMCUCTNode* node,
            unsigned int const& actionIndex,
            double const& initialQValue);

    // Outcome selection
    WMCUCTNode* selectOutcome(WMCUCTNode* node, PDState& nextState, int& varIndex);

    // Backup functions
    void backupDecisionNodeLeaf(WMCUCTNode* node, double const& immReward,
            double const& futReward);
    void backupDecisionNode(WMCUCTNode* node, double const& immReward,
            double const& futReward);
    void backupChanceNode(WMCUCTNode* node, double const& futReward);

    // Memory management
    WMCUCTNode* getRootNode() {
        return getWMCUCTNode(1.0);
    }

    WMCUCTNode* getDummyNode() {
        return getWMCUCTNode(1.0);
    }

private:
    // Memory management
    WMCUCTNode* getWMCUCTNode(double const& prob) {
        WMCUCTNode* res = UCTBase<WMCUCTNode>::getSearchNode();
        res->prob = prob;
        return res;
    }

    // Parameter
    double heuristicWeight;
};

#endif
