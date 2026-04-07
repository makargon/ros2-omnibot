from math import cos, sin

v = [0, 10, 10]

# но это неправильно :/
def check():
    v_x = []
    v_y = []
    for i in range(len(v)):
        v_x.append(v[i] * sin(i*120))
        v_y.append(v[i] * cos(i*120))
    print(sum(v_x))
    print(sum(v_y))