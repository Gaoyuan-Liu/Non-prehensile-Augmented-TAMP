from collections import Iterator, namedtuple, deque

# A stream
def get_gen():
    def stream(x1, x2, x3):
        i=0
        while True:
            y1 = i * (x1 + x2)
            y2 = i * (x2 + x3)
            yield (y1, y2)
            i += 1
    return stream

# Test of from_gen_fn
def from_list_gen_fn(list_gen_fn):
    return list_gen_fn

# The function here is just put the sampled data in a []
def from_gen_fn(gen_fn):
    return from_list_gen_fn(lambda *args, **kwargs: ([] if ov is None else [ov]
                                                     for ov in gen_fn(*args, **kwargs)))

# Test of from_list_fn
INF = float('inf')
class BoundedGenerator(Iterator):
    """
    A generator with a fixed length.
    The generator tracks its number of calls, allowing it to terminate with one fewer call
    """
    def __init__(self, generator, max_calls=INF):
        self.generator = generator
        self.max_calls = max_calls
        self.stopped = False
        self.history = []
    @property
    def calls(self):
        return len(self.history)
    @property
    def enumerated(self):
        return self.stopped or (self.max_calls <= self.calls)
    def next(self):
        if self.enumerated:
            raise StopIteration()
        try:
            self.history.append(next(self.generator))
        except StopIteration:
            self.stopped = True
            raise StopIteration()
        return self.history[-1]
    __next__ = next


def from_list_fn(list_fn):
    #return lambda *args, **kwargs: iter([list_fn(*args, **kwargs)])
    return lambda *args, **kwargs: BoundedGenerator(iter([list_fn(*args, **kwargs)]), max_calls=1)


def main():
    a = from_gen_fn(get_gen())
    print(a)
    b = a(1, 2, 3)


    print(next(b))
    print(next(b))
    print(next(b))
    print(next(b))
    print(next(b))

    # b = a() #stream(1, 2, 3)
    # print(next(b))
    # print(next(b))
    # print(next(b))

if __name__ == '__main__':
    main()