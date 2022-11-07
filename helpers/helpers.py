import asyncio

def run_syncly(coroutine):
    loop = asyncio.get_event_loop()
    loop.run_until_complete(coroutine)

def clamp(x, minimum, maximum):
    return max(minimum, min(x, maximum))

def map(value, istart, istop, ostart, ostop):
    return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))