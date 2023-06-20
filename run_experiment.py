from subprocess import run
from itertools import product
from typing import List

def grid_search(parameters: dict) -> List[dict]:
    for experiment in product(*parameters.values()):
        yield dict(zip(parameters, experiment))


def experiment_to_str(experiment: dict) -> List[str]:
    return list(sum(experiment.items(), start=()))

program = ["./build/lns"]

args = {
        "-m":["./instances/orz900d.map"],
        "-a": ["./scen-even/orz900d-even-1.scen"],
        "-o": ["experiment"],
        "-k": ["1000"],
        "-t": ["1000"],
        "--initAlgo": ["winPP"],
        "--outputPaths": ["paths.txt"],
        "--stats": ["stats.txt"],
        "--timeHorizon": ["100"],
        "--replanningPeriod": ["90", "100"]
}

for index, experiment in enumerate(grid_search(args)):

#    if int(experiment["--timeHorizon"]) < int(experiment["--replanningPeriod"]): continue
    
    command = program + experiment_to_str(experiment)
    print(command)
    run(command) 
    
    
    
    
    #shell=False, capture_output=True, text=True
    # print(result)
