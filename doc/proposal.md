---
title: Integer Programming Using Branch-and-Bound Algorithm.  
author: Cheng Li, Zhangxiaowen Gong, Xuan Wang, Yiyi Wang
date: September 29, 2016
bibliography: proposal.bib
csl: ieee.csl
output: pdf_document
---

# 1. Definitions
An **integer programming** problem is a mathematical optimization or feasibility program in which some or all of the variables are restricted to be integers. In many settings the term refers to **integer linear programming** (ILP), in which the objective function and the constraints (other than the integer constraints) are linear. Integer programming is a NP-hard problem. It can be used in many application areas, such as production planning, scheduling, telecommunications networks, and cellular networks. [^1]  

**Branch and bound** is an algorithm design paradigm for discrete and combinatorial optimization problems, as well as general real valued problems. A branch-and-bound algorithm consists of a systematic enumeration of candidate solutions by means of state space search: the set of candidate solutions is thought of as forming a rooted tree with the full set at the root. The algorithm explores branches of this tree, which represent subsets of the solution set. Before enumerating the candidate solutions of a branch, the branch is checked against upper and lower estimated bounds on the optimal solution, and is discarded if it cannot produce a better solution than the best one found so far by the algorithm.[^2]

[^1]: [wikipedia: Integer programming](https://en.wikipedia.org/wiki/Integer_programming)  
[^2]: [wikipedia: Branch and bound](https://en.wikipedia.org/wiki/Branch_and_bound)

# 2. Parallel programming for branch and bound algorithm.
Large-scale system with distributed memory are a natural fit to solving the computationally complex integer programming problem. As branch-and-bound algorithm uses state space search to explore the desired property, we can divide the tree into many subproblems (or subtrees) and parallelize branching computations [@Bader2004]. Each processors can be responsible for different subproblems. In this way we can highly increase the performance of solving an integer programming problem.        

# 3. Goal  
We plan to leverage the branch-and-bound algorithm to solve integer programming problems in parallel in Charm\+\+. Specifically we target at Travelling Salesman Problem(TSP) as a start point. TSP can be formulated as an integer linear program[@coaac][@odgaip][@lpne]. There are several things that we need to do. First, we would need to implement the Simplex Method for finding the linear programming optimal solution, which is a key part of solving the integer programming problem. Next, we need to define TSP to solve. In the future, we might extend the framework with an interface so that user can define any arbitrary integer programming problem. After that we would implement the branch-and-bound algorithm in Charm++. In the end, we would test our implementation under different scenarios. The LP part would be solved by off-the-shelf sovler like Gurobi[@gurobi], GLPK[@glpk] or Cbc[@cbc]. Some of the challenges include the intelligent pruning of the state space, load balancing, and low-overhead communication between chares. We hope that we could solve those problems at the time we implementing our algorithm.

# 4. References
reference goes here
