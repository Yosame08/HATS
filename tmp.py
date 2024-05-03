tot = 0
mxi = 0
cnt = 0
sec = 0
with open("test_sampled.txt", "r") as f:
  lines = f.readlines()
  for line in lines:
    info = line.strip().split(' ')
    if len(info) == 1:
      now = int(info[0])
      assert tot - 1 == now
      tot = now
      if cnt > mxi:
        sec = mxi
        mxi = cnt
      elif cnt > sec:
        sec = cnt
      cnt = 0
    else:
      cnt += 1
if cnt > mxi:
  mxi = cnt
print(tot)
print(mxi)
print(sec)
