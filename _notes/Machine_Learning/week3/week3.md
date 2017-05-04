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
* **binary classification** : problem in which y can take on only two values, 0 and 1. For instance, if we are trying to build a spam classifier for email, then $$x^{(i)}$$ may be some features of a piece of email, and y may be 1 if it is a piece of spam mail, and 0 otherwise. Hence, $$ y \in \big[ 0,1 \big]$$. 0 is also called the **negative class**, and 1 the **positive class**, and they are sometimes also denoted by the symbols “-” and “+.” Given $$x^{(i)}$$, the corresponding $$y^{(i)}$$ is also called the label for the training example.

### Hypothesis Representation
Instead of using linear regression where the outputs $$ y \in \big[ - \infty,\infty \big]$$, we want to use logistic regression which only generates values  $$ y \in \big[ 0,1 \big]$$ for classification.  To fix this, let’s change the form for our hypotheses $$h_\theta(x)$$ to satisfy $$0\leq h_\theta(x) \leq1$$. This is accomplished by plugging $$\theta^{(T)}x $$ into the **Logistic Function**.

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

### Decision Boundary
The decision boundary is a property of the hypothesis and gives us a sense how the hypothesis makes its prediction. The decision boundary is independent of the data set and is fixed once the parameters are set.

In order to get our discrete 0 or 1 classification, we can translate the output of the hypothesis function as follows:

![sigmoid function]({{site.url}}/notes/Machine_Learning/week3/images/sigmoid2.png)

$$ h_\theta(x) \geq 0.5 \rightarrow y = 1$$

$$h_\theta(x) < 0.5 \rightarrow y = 0$$

The way our logistic function g behaves is that when its input is greater than or equal to zero, its output is greater than or equal to 0.5.

$$ \begin{align*}& g(z) \geq 0.5 \text{ when } z \geq 0 \end{align*} $$

Recall:

$$ z=0, e^{0}=1 \Rightarrow g(z)=1/2 $$

$$ z \to \infty, e^{-\infty} \to 0 \Rightarrow g(z)=1 $$

$$ z \to -\infty, e^{\infty}\to \infty \Rightarrow g(z)=0 $$

So if our input to g is $$ h_\theta(x) $$ , then that means:

$$ \begin{align*}& h_\theta(x) = g(\theta^T x) \geq 0.5  \text{ when } \theta^T x \geq 0 \newline & \theta^T x \geq 0 \Rightarrow y = 1 \newline & \theta^T x < 0 \Rightarrow y = 0 \end{align*} $$

#### Example: Linear 
![sigmoid function]({{site.url}}/notes/Machine_Learning/week3/images/linear_decision_boundary.png)

$$ \begin{align*}& \theta = \begin{bmatrix}-3 \newline 1 \newline 1\end{bmatrix} \newline & y = 1 \; if \; -3 + x_1 + x_2 \geq 0 \newline & x_1 + x_2 \geq 3 \text{ is the decision boundary line} \end{align*} $$

#### Example: Non-Linear
We can create more complex decision boundary with higher order (greater capacity) polynomials.

![sigmoid function]({{site.url}}/notes/Machine_Learning/week3/images/nonlinear_decision_boundary.png)

$$ h_\theta(x) = g(\theta_0 + \theta_1 x_1+ \theta_2 x_1^2 + \theta_3 x_2^2)$$
