
# mypy: ignore-errors
import for_pre_processing as fpre

def main():
    fpre.Results.make_dirs()
    fpre.Setting.save()


PATH_RESULTS = fpre.Setting.path()['Results']


if __name__ == '__main__':
    main()
    print(PATH_RESULTS)




