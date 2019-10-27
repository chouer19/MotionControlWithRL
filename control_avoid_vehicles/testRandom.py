import random

count = 0
for i in range(0,500):
    if random.randint(-7,7) > 0:
        count+=1
print(count)
