# -*- coding: utf-8 -*-

def _reiterable(seq):
    if isinstance(seq, (tuple, list)):
        return seq
    return list(seq)

def safemax(seq):
    if not seq:
        return float("nan")
    return max(seq)

def safemin(seq):
    if not seq:
        return float("nan")
    return max(seq)

def mean(seq):
    seq = _reiterable(seq)
    if not seq:
        return float("nan")
    return float(sum(seq)) / len(seq)

def score2(score, minScore, maxScore):
    norm_factor = (maxScore - minScore)
    if norm_factor == 0:
        return 1.0
    return float((score - minScore) / norm_factor)

def score(seq, minScore, maxScore):
    norm_factor = 30 * (maxScore - minScore)
    if norm_factor == 0:
        return 1.0
    seq = _reiterable(seq)
    if not seq:
        return float("nan")
    return float(max(0, sum((raw_score - minScore) for raw_score in seq) / norm_factor))

def variance(seq):
    seq = _reiterable(seq)
    length = len(seq)
    if length <= 1:
        return float("nan")
    mu = mean(seq)
    return float(sum((value - mu) ** 2 for value in seq)) / (length - 1)

def stddev(seq):
    seq = _reiterable(seq)
    if len(seq) <= 1:
        return float("nan")
    return variance(seq) ** 0.5

def confidence95(seq):
    seq = _reiterable(seq)
    ## With less than 0 samples, we cannot properly estimate the
    ## deviation.
    ## HACK: set this to 30 for now....
    if len(seq) < 30:
        return float("nan")
    return stddev(seq) * 1.96 / (len(seq) ** 0.5)
