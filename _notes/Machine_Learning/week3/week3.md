---
layout: notes
title: "Week 3 - Machine Learning (Stanford)"
tags: [Machine Learning]        
date: 2017-04-30     
---

- Table of Contents
{:toc}

## Classification and Representation

### Classification
The classification problem is just like the regression problem, except that the values y we now want to predict take on only a small number of discrete values. 
* **binary classification** : problem in which y can take on only two values, 0 and 1. For instance, if we are trying to build a spam classifier for email, then $$x^{(i)}$$ may be some features of a piece of email, and y may be 1 if it is a piece of spam mail, and 0 otherwise. Hence, y∈{0,1}. 0 is also called the **negative class**, and 1 the **positive class**, and they are sometimes also denoted by the symbols “-” and “+.” Given $$x^{(i)}$$, the corresponding $$y^{(i)}$$ is also called the label for the training example.

### Hypothesis Representation
Instead of using linear regression where the outputs are not discrete values, we want to use logistic regression which only generates discrete values for classification.  To fix this, let’s change the form for our hypotheses $$h_\theta(x)$$ to satisfy $$0\leq h_\theta(x) \leq1$$. This is accomplished by plugging $$\theta^{(T)}x $$ into the **Logistic Function**.

#### Logistic Function
* The logistic function is also called the **signmoid function** $$g(z)$$:

$$h_\theta (x) = g ( \theta^T x ) $$

$$z = \theta^T x$$

$$g(z) = \dfrac{1}{1 + e^{-z}}$$

The following image shows us what the sigmoid function looks like:

![sigmoid function]({{site.url}}/notes/Machine_Learning/week3/images/sigmoid.png)

The function $$g(z)$$, shown here, maps any real number to the (0, 1) interval, making it useful for transforming an arbitrary-valued function into a function better suited for classification.

#### Interpreting the hypothesis
When our hypothesis $$h_\theta (x)$$ outputs a number, we treat that value as the estimated probability that $$y = 1$$ on input x.

For example, $$h_\theta (x) = 0.7$$ gives us a probability of 70% that our output is 1. Our probability that our prediction is 0 is just the complement of our probability that it is 1 (e.g. if probability that it is 1 is 70%, then the probability that it is 0 is 30%).

Recap on mutual exclusive probabilities:

read as Probability that y=1, given x, parameterized by θ

$$h_\theta(x) = P(y=1 | x ; \theta) = 1 - P(y=0 | x ; \theta) $$

$$P(y = 0 | x;\theta) + P(y = 1 | x ; \theta) = 1 $$