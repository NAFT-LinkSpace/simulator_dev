# mypy: ignore-errors
import pandas as pd
import numpy as np
from pathlib import Path

from IPython.display import display

if __name__ == '__main__':
    import sys
    sys.path.append(str(Path(__file__).parent.parent))

from pre_processing import PATH_RESULTS
PATH_HISTORIES = PATH_RESULTS + '/Histories'