#####################################################################
#   파일 내용 작성
#       - 세부 사항 1
#       - 세부 사항 2
#   written by jslee
#   date : 2000.00.00
#####################################################################


import numpy as np
import open3d as o3d

import numpy as np
import time, os

from multiprocessing import Pool
# import multiprocessing



def work_func(x):
    print('value %s is in PID : %s' % (x, os.getgid()))
    time.sleep(1)
    return x**5


def main():
    start = int(time.time())
    print(list(map(work_func, range(0, 12))))
    print('*** run time (sec) :', int(time.time()) - start)


def multiprocessing_main():
    start = int(time.time())
    num_cores = 4
    pool = Pool(num_cores)
    print(pool.map(work_func, range(1, 13)))
    print('*** run time (sec) :', int(time.time()) - start)

if __name__ == '__main__':

    main()
    multiprocessing_main()

    print('Done')