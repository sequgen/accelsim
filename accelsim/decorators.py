import numpy as np
import functools

def arrayify(f):
    '''Arrayify a function from R into R^n'''

    def ensure_iterable(x):
        '''Coerces single number x into the one-element iterator (x, )'''
        return (x,) if not hasattr(x, '__iter__') else x

    @functools.wraps(f)
    def vectorized_f(ts, *args, **kwargs):
        # Useful for uniformity. Otherwise, calling with a double or integer would fail
        ts = ensure_iterable(ts)
        results = map(lambda t: f(t, *args, **kwargs), ts)

        # Create tidy output
        results_list = list(results)  # List of matrices
        results_array = np.asarray(results_list).transpose()[0]

        return results_array

    return vectorized_f

# TODO: check if this can be done more elegantly with itertools, or at least merge this decorator in one
def arrayify_abs(f):
    '''Arrayify a function from R into R'''

    def ensure_iterable(x):
        '''Coerces single number x into the one-element iterator (x, )'''
        return (x,) if not hasattr(x, '__iter__') else x

    @functools.wraps(f)
    def vectorized_f(ts, *args, **kwargs):
        # Useful for uniformity. Otherwise, calling with a double or integer would fail
        ts = ensure_iterable(ts)
        results = map(lambda t: f(t, *args, **kwargs), ts)

        # Create tidy output
        results_list = list(results)  # List of matrices
        # <-- This is the only difference with arrayify
        results_array = np.asarray(results_list).transpose()

        return results_array

    return vectorized_f
