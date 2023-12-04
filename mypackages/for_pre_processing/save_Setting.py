# mypy: ignore-errors
import shutil
from pathlib import Path

if __name__ == '__main__':
    import sys
    sys.path.append(str(Path(__file__).parent.parent))

    from for_pre_processing.input_from_Setting import set_path
    from for_pre_processing.make_dirs import Manage_Dirs_Results

else:
    from .input_from_Setting import set_path
    from .make_dirs import Manage_Dirs_Results


PATH_SETTING = set_path()['PATH_SETTING']

class Save_Setting:

    def __init__ (self, path):
        self.path_from = path
        self.path_results = Manage_Dirs_Results.root_path()
        self.path_to = self.path_results + '/Copy_Setting'

    def save(self):

        if __name__ == '__main__':
            Manage_Dirs_Results.make_dirs()

        shutil.copytree(self.path_from, self.path_to, dirs_exist_ok=True)

    def path(self):

        return {'Setting' : self.path_from, 'Results' : self.path_results}


Save_Setting = Save_Setting(PATH_SETTING)





