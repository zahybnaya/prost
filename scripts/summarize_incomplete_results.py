#! /usr/bin/env python
# -*- coding: latin-1 -*-

# This script creates a summary file (summary.res) for all directories
# contained in resultsDir, compresses the contained files
# (results.bz2) and deletes the original ones

import os
import sys
import tarfile
import shutil
import statistics

benchmarkSet = "IPPC2011"
numberOfRuns = 30 #TODO: Derive this automatically!

def summarizeResultsOfDir(dirName, plannerName):
    if not os.path.isdir(dirName):
        return

    files = os.listdir(dirName)
    if "result.xml" in files:
        return

    print "parsing results in " + dirName + "..."

    results = dict()
    times = dict()
    timesPerDomain = dict()
    total_time = float(0.0)
    roundByRoundResults = dict()

    for fileName in files:
        if fileName.endswith(".log") and not fileName.endswith("server.log"):
            errorFile = fileName[:-4]+".err"
            if os.path.getsize(dirName+"/"+errorFile) != 0:
                continue
            else:
                res,time,round_by_round = parseLogFile(dirName+"/"+fileName)
                if res is None:
                    print "Cannot read average reward in " + dirName + "/" + fileName + "!"
                    continue
                if time is None:
                    print "Cannot read total time in " + dirName + "/" + fileName + "!"
                    continue
                if len(round_by_round) != numberOfRuns:
                    print "Missing round by round results in " + dirName + "/" + fileName + "!"
                    continue
                domainName = fileName.split("_")[0].split("/")[-1]
                if not domainName in results:
                    results[domainName] = dict()
                    times[domainName] = dict()
                    timesPerDomain[domainName] = float(0.0)
                    roundByRoundResults[domainName] = dict()
                problemNumber = fileName.split("__")[1].split(".")[0]
                assert not problemNumber in results[domainName]
                results[domainName][problemNumber] = res
                times[domainName][problemNumber] = time
                timesPerDomain[domainName] += time
                total_time += time
                roundByRoundResults[domainName][problemNumber] = round_by_round

    writeResultSummary(dirName, plannerName, results, times, timesPerDomain, total_time, roundByRoundResults)

def writeResultSummary(dirName, plannerName, results, times, timesPerDomain, total_time, roundByRoundResults):
    reslist = list()
    reslist.append("<?xml version=\"1.0\"?>")
    reslist.append("<PlannerResult>")
    reslist.append("\t<PlannerName>"+plannerName+"</PlannerName>")
    reslist.append("\t<BenchmarkSet>"+benchmarkSet+"</BenchmarkSet>")
    reslist.append("\t<NumberOfRuns>"+str(numberOfRuns)+"</NumberOfRuns>")
    reslist.append("\t<Time>"+str(total_time)+"</Time>")
    reslist.append("")

    for domain in results:
        reslist.append("\t<Domain>")
        reslist.append("\t\t<DomainName>"+domain+"</DomainName>")
        reslist.append("\t\t<Time>"+str(timesPerDomain[domain])+"</Time>")
        reslist.append("")
        for problem in results[domain]:
            reslist.append("\t\t<Problem>")
            reslist.append("\t\t\t<ProblemName>"+problem+"</ProblemName>")
            reslist.append("\t\t\t<AvgReward>"+str(results[domain][problem])+"</AvgReward>")
            reslist.append("\t\t\t<Confidence95>"+str(round(statistics.confidence95(roundByRoundResults[domain][problem]),2))+"</Confidence95>")
            reslist.append("\t\t\t<Time>"+str(times[domain][problem])+"</Time>")
            reslist.append("\t\t</Problem>")
            reslist.append("")
        reslist.append("\t</Domain>")
        reslist.append("")
    reslist.append("</PlannerResult>")

    f = open(dirName+"/"+"result.xml", 'w+')

    for entry in reslist:
        f.write(entry+"\n")
    f.close()

    print "...parsing results finished."
        
    print dirName + "/results.xml s created.\n"

def parseLogFile(fileName):
    f = open(fileName)
    rest = tail(f,3)
    rest = rest.split("\n")[0];
    rest = rest.split(":")
    reward = None
    if len(rest) == 2 and rest[0] == ">>>          AVERAGE REWARD":
        reward = rest[1]
        reward = round(float(reward),2)
        

    f.close()

    f = open(fileName)
    rest = tail(f,1)
    rest = rest.strip()[0:-1]
    time = None
    if rest.startswith("PROST complete running time:"):
        rest = rest.split(" ")
        time = rest[-1]
        time = round(float(time),2)

    f.close()

    f = open(fileName)
    roundByRoundResults = list()
    for line in f:
        if line.startswith(">>> END OF ROUND"):
            line = line.split(" ")
            rbrres = round(float(line[-1]),2)
            roundByRoundResults.append(rbrres)

    f.close()

    return reward, time, roundByRoundResults

def tail(f, window=20):
    BUFSIZ = 1024
    f.seek(0, 2)
    bytes = f.tell()
    size = window
    block = -1
    data = []
    while size > 0 and bytes > 0:
        if (bytes - BUFSIZ > 0):
            # Seek back one whole BUFSIZ
            f.seek(block*BUFSIZ, 2)
            # read BUFFER
            data.append(f.read(BUFSIZ))
        else:
            # file too small, start from begining
            f.seek(0,0)
            # only read what was not read
            data.append(f.read(bytes))
        linesFound = data[-1].count('\n')
        size -= linesFound
        bytes -= BUFSIZ
        block -= 1
    return '\n'.join(''.join(data).splitlines()[-window:])

def summarizeIncompleteResults(directory) :

    resultDirNames = os.listdir(directory)
    for resultDirName in resultDirNames:
        summarizeResultsOfDir(directory+resultDirName, resultDirName.replace("_"," "))

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print >> sys.stderr, "Usage: summarize_incomplete_results.py <resultsDir>"
        exit()

    summarizeIncompleteResults(sys.argv[1])

