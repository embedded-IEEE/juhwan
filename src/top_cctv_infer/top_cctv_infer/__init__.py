import os


def get_path(file_name=None):
    p = os.path.dirname(os.path.abspath(__file__)).split("/")
    LIB_PATH = os.path.join("/", *p[1:4], "src", *p[5:6], *p[5:6], "lib", file_name)
    return LIB_PATH