from typing import Optional
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import os

app = FastAPI()

origins = ["*"]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/power")
def get_power():
    os.system("irsend SEND_ONCE midea POWER")
    return {"message": "ok"}       

@app.get("/forwards")
def get_forwards():
    os.system("irsend SEND_ONCE midea FORWARD")
    return {"message": "ok"}       
    
@app.get("/backwards")    
def get_backwards():
    os.system("irsend SEND_ONCE midea BACKWARD")
    return {"message": "ok"}
    
@app.get("/left")    
def get_left():
    os.system("irsend SEND_ONCE midea LEFT")
    return {"message": "ok"}
    
@app.get("/right")    
def get_right():
    os.system("irsend SEND_ONCE midea RIGHT")
    return {"message": "ok"}
    
@app.get("/charge")    
def get_charge():
    os.system("irsend SEND_ONCE midea CHARGE")
    return {"message": "ok"}
    
@app.get("/program")    
def get_program():
    os.system("irsend SEND_ONCE midea PROGRAM")
    return {"message": "ok"}

@app.get("/vacuum")    
def get_vacuum():
    os.system("irsend SEND_ONCE midea VACUUM")
    return {"message": "ok"}
