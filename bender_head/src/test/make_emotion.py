def Happy(e):
  motor = [ , , , ]
  mouth = []
  color = []
  print("Bender" + " " + e)
  return color, motor, mouth

def Sad(e):
  motor = []
  mouth = []
  print("Bender" + " " + e)
  return motor, mouth

def Neutral(e):
  motor = []
  mouth = []
  print("Bender" + " " + e)
  return motor, mouth

def Angry(e):
  motor = []
  mouth = []
  print("Bender" + " " + e)
  return motor, mouth

def Surprise(e):
  motor = []
  mouth = []
  print("Bender" + " " + e)
  return motor, mouth

def expression(e):
    try: 
      globals()[e](e)
    except:
      print("ERROR")
