import time
from functools import wraps


# In future, we may want to define a separate throttle duration to the
# function argument.
def throttle(fn):
    """
    Decorator for a function that requires throttling - i.e.
    prevented from being called again within a given duration.

    Because the main Dalek control runs on a continuous loop, triggers
    responding to various states would get called over and over.

    It is expected that the first argument to the decorated `fn` will be
    the throttle duration.

    Example:

    @throttle
    def do_something_for_a_while(duration_in_seconds):
        turn_on()
        wait(duration_in_seconds)
        turn_off()
    """
    last_call = None

    @wraps(fn)
    def wrapped_fn(duration_in_seconds, *args, **kwargs):
        nonlocal last_call

        if last_call and time.time() - last_call < duration_in_seconds:
            return None
        last_call = time.time()
        return fn(duration_in_seconds, *args, **kwargs)

    return wrapped_fn
