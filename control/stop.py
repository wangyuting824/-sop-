"""Call the adapter's verified stop operation or propagate Unsupported."""
def stop(adapter):
    return adapter.stop_motion()
