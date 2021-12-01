boxes = []
w = 1.0
h = 1.0
dx = w/4
dy = h/4
area = 1.5    # 2m by 2m area
num_steps = int(area / dy) - 1
if num_steps <= 0:
    num_steps = 1
print(num_steps)
for i in range(num_steps):
    for yi in range(-i,i+1):
        for xi in range(-i,i+1):
            box = [-w/2+xi*dx, w/2+xi*dx, -h/2+yi*dy, h/2+yi*dy]
            if box not in boxes:
                boxes.append(box)
print(len(boxes))

# area = 3m x 3m: 3m-1m = 2m/0.25 = 8+1 =9*9 = 81
# ((x-1)/0.25 + 1)^2