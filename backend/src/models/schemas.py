"""Pydantic schemas for request/response validation"""

from pydantic import BaseModel, EmailStr
from typing import Optional, List
from datetime import datetime


class UserBase(BaseModel):
    """Base user schema"""

    email: EmailStr
    full_name: str


class UserCreate(UserBase):
    """User creation schema"""

    password: str
    hardware_profile: str  # "rtx" or "jetson"


class User(UserBase):
    """User response schema"""

    id: int
    hardware_profile: str
    created_at: datetime

    class Config:
        from_attributes = True


class ChatMessage(BaseModel):
    """Chat message schema"""

    id: int
    user_id: int
    query: str
    response: str
    sources: List[str]
    created_at: datetime

    class Config:
        from_attributes = True


class ChatRequest(BaseModel):
    """Chat request schema"""

    query: str


class ChatResponse(BaseModel):
    """Chat response schema"""

    answer: str
    sources: List[dict]  # List of {title, url, excerpt}


class HealthResponse(BaseModel):
    """Health check response"""

    status: str
    database: str
    qdrant: str
