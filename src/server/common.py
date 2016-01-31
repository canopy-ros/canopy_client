
clients = dict()


def get_client(name):
    return clients[name]


def add_client(name, client):
    global clients
    clients[name] = client


def remove_client(name):
    global clients
    del clients[name]
