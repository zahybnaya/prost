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
        futureReward(-std::numeric_limits<double>::max()),
	ci(0.0),
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
        solved = false;
        rewardLock = false;
	ci = 0.0;
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

    std::vector<CDPUCTNode*> children;

private:
    double immediateReward;
    double futureReward;
    double ci;
    int numberOfVisits;

    double prob;
    bool solved;
    bool rewardLock;
};

class CDPUCTSearch : public UCTBase<CDPUCTNode> {
public:
    CDPUCTSearch() :
        UCTBase<CDPUCTNode>("CDP-UCT"),
	tests(0),
	updates(0),
	debugMessage(false),
	//visitsThreshold(0),
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

   /* virtual void getUpdateRate() {
	double rate = (double)updates / (double)tests;
	updates = 0;
	tests = 0;

	std::cout << "Update rate: " << rate << std::endl;
    }*/

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
   int tests;
   int updates;
   bool debugMessage;
   // int visitsThreshold;

    // Memory management
    CDPUCTNode* getCDPUCTNode(double const& prob) {
        CDPUCTNode* res = UCTBase<CDPUCTNode>::getSearchNode();
        res->prob = prob;
        return res;
    }

    // Parameter
    double heuristicWeight;

    //Helper math functions
    int oneTailTTest(CDPUCTNode* node1, CDPUCTNode* node2);
};

#endif
