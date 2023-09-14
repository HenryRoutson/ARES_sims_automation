


# Sims optimisation at ARES


Specifically we need to automate optimising ballast (weight ditribution) in our rocket to reach 30k feet. But for the sake of other possible optimsation purposes, this will attempt to be generalised.

# General process


```

REPEAT
1 Simulate rocket flight path in rocketpy (or other accurate software)
2 Change rocket design to get closer to ideal outcome

```



```Python

def iterate(rocket_path, rocket_design):
  # depending on rocket_path
  # change rocket_design

  return new_rocket_design




rocket_design = ... # initial rocket design

while true:
  rocket_path = simulate_rocket(rocket_design)
  rocket_design = iterate(rocket_path, rocket_design)

```



# Required software

For our use case we will be using 
```
  OpenRocket   for calculating values such as center of mass
  Rocketpy     for accurate rocket_path simulation
```

This unfortunatly restricts us from using colab to avoid machine specific setup issues,
