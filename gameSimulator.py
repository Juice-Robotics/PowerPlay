
def scoreMatch(auton, ground, low, mid, high):
    score = 0
    score += auton * 5
    score += ground * 2
    score += low * 3
    score += mid * 4
    score += (high + auton) * 5
    return score


def runSim(scenario):
    field = [
        {
           "junction": "LOW",
           "cones": 0,
       }
    ]
    return scoreMatch(scenario["auto"], 0, (10*(scenario["coneSuccessRate"]/100)), (10*(scenario["coneSuccessRate"]/100)), (10*(scenario["coneSuccessRate"]/100)))

scenarios = [
    {
        "auto": 6,
        "coneSuccessRate": 100
    },

    {
        "auto": 6,
        "coneSuccessRate": 80
    }
]

print(runSim(scenarios[0]))
print(runSim(scenarios[1]))