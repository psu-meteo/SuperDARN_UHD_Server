# https://www.andreas-jung.com/contents/a-python-decorator-for-measuring-the-execution-time-of-methods 
import time


def timeit(method):

    def timed(*args, **kw):
        ts = time.time()
        result = method(*args, **kw)
        te = time.time()

        print('--------  %r (%r, %r) %2.5f sec' % (method.__name__, args, kw, te-ts))
        return result

    return timed

