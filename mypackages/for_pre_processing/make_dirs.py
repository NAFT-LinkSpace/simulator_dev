# mypy: ignore-errors
from datetime import datetime
from pathlib import Path


class Manage_Dirs:

    def __init__ (self):

        now = datetime.now()
        self.TIME_NOW = now.strftime('%Y-%m%d-%H%M%S')

        self.PATH_TOP = Path(__file__).parent.parent.parent
        self.PATH_RESULTS = self.PATH_TOP / '4_Results'
        self.SUB_DIRS = ['Graphs', 'Summaries_History', 'Histories', 'Copy_Setting']

        self.LIST_PATH_str = [(self.PATH_RESULTS / str(self.TIME_NOW) / str(path)).as_posix() for path in self.SUB_DIRS]
        self.LIST_PATH_obj = [self.PATH_RESULTS / str(self.TIME_NOW) / str(path) for path in self.SUB_DIRS]

    def make_dirs(self):

        for path in self.LIST_PATH_str:
            Path(path).mkdir(parents=True, exist_ok=True)

    def root_path(self):

        return (self.PATH_RESULTS / str(self.TIME_NOW)).as_posix()

    def list_str(self):

        return self.LIST_PATH_str

    def list_obj(self):

        return self.LIST_PATH_obj


Manage_Dirs_Results = Manage_Dirs()

