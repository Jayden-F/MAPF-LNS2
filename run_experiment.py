from concurrent.futures import ProcessPoolExecutor
import subprocess
from itertools import product, chain
from typing import List, Dict
import glob 

def linear_product(parameters: Dict[str, List[str]]) -> List[str]:
    for experiment in product(*parameters.values()):
        yield list(chain(*zip(parameters, experiment)))

def run(arguments: List[str]):
    print(*arguments)
    return subprocess.call(arguments, shell=True)


maps: List[str] = glob.glob("./map/orz*.map")  
scenarios: List[str] = glob.glob("./scenario/orz*.scen")
print(scenarios)

# Format the following as a dictionary of lists of strings
args = {
    "./build/lns": [""],
    "-m": maps,
    "-a": scenarios,
    "-o": ["test"],
    "-k": ["200"],
    "-t": ["30"],
    "--initLNS": ["false"],
    "--initAlgo": ["winPP"],
    "--outputPaths": ["paths.txt"],
    "--planningPeriod": ["10"],
    "--planningHorizon": ["20"],
    "--screen": ["0"]
}

for params in linear_product(args):
    print(*params)

with ProcessPoolExecutor(max_workers=1) as executor:
    executor.map(run, linear_product(args))