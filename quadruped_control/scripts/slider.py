from multiprocessing import Process, Manager
manager = Manager()
shared_variables = manager.Array('f', [1,1,1, 0,0,0, 0,0,0.425 ])


if __name__ == "__main__":
    # construct a different process for each function
    max_size = 1000000000
    processes = [Process(target=add, args=(range(1, max_size), range(1, max_size))),
                 Process(target=sub, args=(range(1, max_size), range(1, max_size))),
                 Process(target=mult, args=(range(1, max_size), range(1, max_size)))]

    # kick them off 
    for process in processes:
        process.start()