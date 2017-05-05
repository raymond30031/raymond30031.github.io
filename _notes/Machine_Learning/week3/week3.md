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
![linear_decision_boundary]({{site.url}}/notes/Machine_Learning/week3/images/linear_decision_boundary.png)

$$ \begin{align*}& \theta = \begin{bmatrix}-3 \newline 1 \newline 1\end{bmatrix} \newline & y = 1 \; if \; -3 + x_1 + x_2 \geq 0 \newline & x_1 + x_2 \geq 3 \text{ is the decision boundary line} \end{align*} $$

#### Example: Non-Linear
We can create more complex decision boundary with higher order (greater capacity) polynomials.

![nonlinear_decision_boundary]({{site.url}}/notes/Machine_Learning/week3/images/nonlinear_decision_boundary.png)

$$ h_\theta(x) = g(\theta_0 + \theta_1 x_1+ \theta_2 x_1^2 + \theta_3 x_2^2)$$

## Logistic Regression Model
### Cost Function
Defines the objective for the optimization. Cost function for logistic regression:

$$ \begin{align*}& J(\theta) = \dfrac{1}{m} \sum_{i=1}^m \mathrm{Cost}(h_\theta(x^{(i)}),y^{(i)}) \newline & \mathrm{Cost}(h_\theta(x),y) = -\log(h_\theta(x)) \; & \text{if y = 1} \newline & \mathrm{Cost}(h_\theta(x),y) = -\log(1-h_\theta(x)) \; & \text{if y = 0}\end{align*} $$

![cost-1]({{site.url}}/notes/Machine_Learning/week3/images/cost_1.png)
![cost_0]({{site.url}}/notes/Machine_Learning/week3/images/cost_0.png)

$$ \begin{align*}& \mathrm{Cost}(h_\theta(x),y) = 0 \text{ if } h_\theta(x) = y \newline & \mathrm{Cost}(h_\theta(x),y) \rightarrow \infty \text{ if } y = 0 \; \mathrm{and} \; h_\theta(x) \rightarrow 1 \newline & \mathrm{Cost}(h_\theta(x),y) \rightarrow \infty \text{ if } y = 1 \; \mathrm{and} \; h_\theta(x) \rightarrow 0 \newline \end{align*} $$

If our correct answer 'y' is 0, then the cost function will be 0 if our hypothesis function also outputs 0. If our hypothesis approaches 1, then the cost function will approach infinity.

If our correct answer 'y' is 1, then the cost function will be 0 if our hypothesis function outputs 1. If our hypothesis approaches 0, then the cost function will approach infinity.

Note that writing the cost function in this way guarantees that J(θ) is convex for logistic regression.

### Simplified Cost Function and Gradient Descent
We can compress our cost function's two conditional cases into one case:

$$ \mathrm{Cost}(h_\theta(x),y) = - y \; \log(h_\theta(x)) - (1 - y) \log(1 - h_\theta(x)) $$

Notice that when y is equal to 1, then the second term (1−y)log(1−hθ(x)) will be zero and will not affect the result. If y is equal to 0, then the first term −ylog(hθ(x)) will be zero and will not affect the result.

We can fully write out our entire cost function as follows:

$$ J(\theta) = - \frac{1}{m} \displaystyle \sum_{i=1}^m [y^{(i)}\log (h_\theta (x^{(i)})) + (1 - y^{(i)})\log (1 - h_\theta(x^{(i)}))] $$

A vectorized implementation is:

$$ \begin{align*} & h = g(X\theta)\newline & J(\theta) = \frac{1}{m} \cdot \left(-y^{T}\log(h)-(1-y)^{T}\log(1-h)\right) \end{align*} $$

Remember that the general form of gradient descent is:

$$ \begin{align*}& Repeat \; \lbrace \newline & \; \theta_j := \theta_j - \alpha \dfrac{\partial}{\partial \theta_j}J(\theta) \newline & \rbrace\end{align*} $$

We can work out the derivative part using calculus to get:

$$ \begin{align*} & Repeat \; \lbrace \newline & \; \theta_j := \theta_j - \frac{\alpha}{m} \sum_{i=1}^m (h_\theta(x^{(i)}) - y^{(i)}) x_j^{(i)} \newline & \rbrace \end{align*} $$

Notice that this algorithm is identical to the one we used in linear regression. We still have to simultaneously update all values in theta.

A vectorized implementation is:

$$ \theta := \theta - \frac{\alpha}{m} X^{T} (g(X \theta ) - \vec{y}) $$

### Advanced Optimization
Advanced concepts for minimizing the cost function for large machine learning problems (e.g. huge feature set)

* Conjugate gradient
* BFGS (Broyden-Fletcher-Goldfarb-Shanno)
* L-BFGS (Limited memory - BFGS)

+ Advantages:
  - No need to manually pick alpha (learning rate). Have a clever inner loop (line search algorithm) which tries a bunch of alpha values and picks a good one.
  - Often faster than gradient descent
  - Can be used successfully without understanding their complexity
+ Disadvantages:
  - Could make debugging more difficult
  - Different libraries may use different implementations and cause different performances and ressults

## Multiclass Classification: One-vs-all
For multiclass classification, we divide the problem into n+1 (+1 because the index starts at 0) binary classification problems; in each one, we predict the probability that 'y' is a member of one of our classes.

$$ \begin{align*}& y \in \lbrace0, 1 ... n\rbrace \newline& h_\theta^{(0)}(x) = P(y = 0 | x ; \theta) \newline& h_\theta^{(1)}(x) = P(y = 1 | x ; \theta) \newline& \cdots \newline& h_\theta^{(n)}(x) = P(y = n | x ; \theta) \newline& \mathrm{prediction} = \max_i( h_\theta ^{(i)}(x) )\newline\end{align*} $$

![multiclassification]({{site.url}}/notes/Machine_Learning/week3/images/multiclassification.png)

We are basically choosing one class and then lumping all the others into a single second class. We do this repeatedly, applying binary logistic regression to each case, and then use the hypothesis that returned the highest value as our prediction.

## Regularization

### Problem of Overfitting
![fitting example]({{site.url}}/notes/Machine_Learning/week3/images/fitting_example.png)

Underfitting, or high bias, is when the form of our hypothesis function h maps poorly to the trend of the data. It is usually caused by a function that is too simple or uses too few features. 

At the other extreme, overfitting, or high variance, is caused by a hypothesis function that fits the available data but does not generalize well to predict new data. It is usually caused by a complicated function that creates a lot of unnecessary curves and angles unrelated to the data.

There are two main options to address the issue of overfitting:

+ Reduce the number of features:
  - Manually select which features to keep.
  - Use a model selection algorithm (studied later in the course).

+ Regularization
  - Keep all the features, but reduce the magnitude of parameters θj.
  -Regularization works well when we have a lot of slightly useful features.

### Cost Function
If we have overfitting from our hypothesis function, we can reduce the weight that some of the terms in our function carry by increasing their cost.

For example: $$ \theta_0 + \theta_1x + \theta_2x^2 + \theta_3x^3 + \theta_4x^4 $$

Without actually getting rid of these features or changing the form of our hypothesis, we can instead modify our cost function: 

$$ min_\theta\ \dfrac{1}{2m}\sum_{i=1}^m (h_\theta(x^{(i)}) - y^{(i)})^2 + 1000\cdot\theta_3^2 + 1000\cdot\theta_4^2 $$

After inflating the cost of $$ \theta_3 $$ and $$ \theta_4 $$, in order for the cost function to get close to zero, it will have to reduce the values of $$ \theta_3 $$ and $$ \theta_4 $$ to near zero. As a result, we see that the new hypothesis fits better and (depicted by the pink curve) looks like a quadratic function despite having higher order terms.

![inflation]({{site.url}}/notes/Machine_Learning/week3/images/inflation.png)

We could also regularize all of our theta parameters in a single summation as:

$$ min_\theta\ \dfrac{1}{2m}\  \sum_{i=1}^m (h_\theta(x^{(i)}) - y^{(i)})^2 + \lambda\ \sum_{j=1}^n \theta_j^2 $$

The lambda, is the regularization parameter. It determines how much the costs of our theta parameters are inflated.

Using the above cost function with the extra summation, we can smooth the output of our hypothesis function to reduce overfitting. If lambda is chosen to be too large, it may smooth out the function too much and cause underfitting.

### Regularized Linear Regression
#### Gradient Descent

We will modify our gradient descent function to separate out $$\theta_0 $$ from the rest of the parameters because we do not want to penalize $$\theta_0 $$.

$$ \begin{align*} & \text{Repeat}\ \lbrace \newline & \ \ \ \ \theta_0 := \theta_0 - \alpha\ \frac{1}{m}\ \sum_{i=1}^m (h_\theta(x^{(i)}) - y^{(i)})x_0^{(i)} \newline & \ \ \ \ \theta_j := \theta_j - \alpha\ \left[ \left( \frac{1}{m}\ \sum_{i=1}^m (h_\theta(x^{(i)}) - y^{(i)})x_j^{(i)} \right) + \frac{\lambda}{m}\theta_j \right] &\ \ \ \ \ \ \ \ \ \ j \in \lbrace 1,2...n\rbrace\newline & \rbrace \end{align*} $$

With some manipulation our update rule can also be represented as:

$$ \theta_j := \theta_j(1 - \alpha\frac{\lambda}{m}) - \alpha\frac{1}{m}\sum_{i=1}^m(h_\theta(x^{(i)}) - y^{(i)})x_j^{(i)}$$

The first term in the above equation, $$1 - \alpha\frac{\lambda}{m}$$ will always be less than 1. Intuitively you can see it as reducing the value of $$\theta_j$$ by some amount on every update. Notice that the second term is now exactly the same as it was before.

#### Normal Equation
$$ \begin{align*}& \theta = \left( X^TX + \lambda \cdot L \right)^{-1} X^Ty \newline& \text{where}\ \ L = \begin{bmatrix} 0 & & & & \newline & 1 & & & \newline & & 1 & & \newline & & & \ddots & \newline & & & & 1 \newline\end{bmatrix}\end{align*} $$

The first element of L is 0 to exclude regularizatino on the intercept term

Recall that if m < n, then $$X^TX$$ is non-invertible. However, when we add the term λ⋅L, then $$X^TX$$ + λ⋅L becomes invertible.

### Regularized Logistic Regression

We can regularize the cost function by adding a term to the end:

$$ J(\theta) = - \frac{1}{m} \sum_{i=1}^m \large[ y^{(i)}\ \log (h_\theta (x^{(i)})) + (1 - y^{(i)})\ \log (1 - h_\theta(x^{(i)}))\large] + \frac{\lambda}{2m}\sum_{j=1}^n \theta_j^2 $$

#### Gradient Descent

$$ \begin{align*} & \text{Repeat}\ \lbrace \newline & \ \ \ \ \theta_0 := \theta_0 - \alpha\ \frac{1}{m}\ \sum_{i=1}^m (h_\theta(x^{(i)}) - y^{(i)})x_0^{(i)} \newline & \ \ \ \ \theta_j := \theta_j - \alpha\ \left[ \left( \frac{1}{m}\ \sum_{i=1}^m (h_\theta(x^{(i)}) - y^{(i)})x_j^{(i)} \right) + \frac{\lambda}{m}\theta_j \right] &\ \ \ \ \ \ \ \ \ \ j \in \lbrace 1,2...n\rbrace\newline & \rbrace \end{align*} $$

$$h_\theta(x) = \dfrac{1}{1 + e^{-\theta^T x}}$$