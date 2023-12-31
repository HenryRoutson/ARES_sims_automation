


# Sims optimisation at ARES


Specifically we need to automate optimising ballast (weight ditribution) in our rocket to reach 30k feet. But for the sake of other possible optimsation purposes, this will attempt to be generalised.

TODO what else needs to be optimised

# General process


```

REPEAT
1 Simulate rocket flight path in rocketpy (or other accurate software)
2 Change rocket design to get closer to ideal outcome

```


## Basic iterate 


```Python

def iterate(outcome, design):
  # depending on outcome
  # change design

  # could be manual

  return new_design




design = ... # initial rocket design

while true:
  outsome = simulate_rocket(design)
  design = iterate(outcome, design)

```




## + Store rocket designs and their associated score to graph iterate function accuracy




```Python




def score_rocket_path(rocket_path):
  # calculate score

  return score


def iterate(rocket_path, rocket_design):
  # depending on rocket_path
  # change rocket_design

  return new_rocket_design





rocket_scores = [] 
rocket_designs = [] 
rocket_design = ... # initial rocket design

while true:
  rocket_path = simulate_rocket(rocket_design)
  rocket_design = iterate(rocket_path, rocket_design)
  score = score_rocket_path(rocket_path)

  rocket_scores.append(score)
  rocket_designs.append(rocket_design)
 
  graph(rocket_scores, rocket_designs) 

```



In our case


```Python
def score_rocket_path(rocket_path):

  return  rocket_path.height_in_feet - 30_000     # return offset from target
```





## + JSON save and load data to avoid needing to keep device on


```Python

import json


def score_rocket_path(rocket_path):
  # calculate score

  return score


def iterate(rocket_path, rocket_design):
  # depending on rocket_path
  # change rocket_design

  return new_rocket_design



# default '[[], [], []]'

(rocket_scores, rocket_designs, rocket_design) = json.loads('[[], [], []]')

while true:
  rocket_path = simulate_rocket(rocket_design)
  rocket_design = iterate(rocket_path, rocket_design)
  score = score_rocket_path(rocket_path)

  rocket_scores.append(score)
  rocket_designs.append(rocket_design)
 
  graph(rocket_scores, rocket_designs) 

print(json.dumps())  # just use strings, files are overkill


```



## 






# Required software

For our use case we will be using 
```
  OpenRocket   for calculating values such as center of mass
  Rocketpy     for accurate rocket_path simulation
```

Unfortunatly OpenRocket restricts us from using colab to avoid machine specific setup issues,







# Example rocket py script

```Python



```