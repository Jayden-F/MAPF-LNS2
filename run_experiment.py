from concurrent.futures import ProcessPoolExecutor
import subprocess
from tqdm import tqdm
from itertools import product, chain
from os import getpid
from typing import List, Dict
import glob 

def linear_product(parameters: Dict[str, List[str]]) -> List[str]:
    retval = []
    for experiment in product(*parameters.values()):
        retval.append( list(chain(*zip(parameters, experiment))))
    return retval

def run(arguments: List[str]):
    print(f"Process {getpid()}: running {' '.join(arguments)}")
    subprocess.run(arguments, stdout=subprocess.DEVNULL)


maps: List[str] = glob.glob("./map/orz*.map")  
scenarios: List[str] = glob.glob("./scenario/orz*.scen")

# Format the following as a dictionary of lists of strings
args = {
    "./build/lns": [""],
    "-m": maps,
    "-a": scenarios,
    "-o": ["test"],
    "-k": ["10"],
    "-t": ["90"],
    "--initLNS": ["false"],
    "--initAlgo": ["winPP"],
    "--outputPaths": ["paths.txt"],
    "--planningPeriod": ["10"],
    "--planningHorizon": ["20"],
    "--screen": ["0"]
}

if __name__ == "__main__":
    experiments: List[str]  = linear_product(args)
    with ProcessPoolExecutor(max_workers=2) as executor:
        result = list(tqdm(executor.map(run, experiments), total=len(experiments), desc="Running experiments"))
