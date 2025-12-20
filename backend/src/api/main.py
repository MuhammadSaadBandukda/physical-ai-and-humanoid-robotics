"""Main FastAPI application"""

import sys
from pathlib import Path

# Add backend directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager
from src.core.config import settings
from src.models.schemas import HealthResponse, ChatRequest, ChatResponse

# Initialize Qdrant client
qdrant_client = None


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Manage application lifecycle"""
    global qdrant_client
    try:
        from qdrant_client import QdrantClient

        qdrant_client = QdrantClient(url=settings.qdrant_url)
        print("Connected to Qdrant")
    except Exception as e:
        print(f"Warning: Could not connect to Qdrant: {e}")
    yield
    # Cleanup if needed
    if qdrant_client:
        qdrant_client = None


app = FastAPI(
    title=settings.app_name,
    description="Backend API for Physical AI & Humanoid Robotics Textbook",
    version="0.1.0",
    lifespan=lifespan,
)

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/health", response_model=HealthResponse)
async def health_check() -> HealthResponse:
    """Health check endpoint

    Returns:
        Status of the application and its dependencies
    """
    db_status = "ok"
    qdrant_status = "ok"

    # Check database connection (simplified)
    try:
        # In a real implementation, this would query the database
        pass
    except Exception as e:
        db_status = f"error: {str(e)}"

    # Check Qdrant connection
    try:
        if qdrant_client:
            qdrant_client.get_collections()
        else:
            qdrant_status = "not connected"
    except Exception as e:
        qdrant_status = f"error: {str(e)}"

    return HealthResponse(
        status="ok" if db_status == "ok" and qdrant_status == "ok" else "degraded",
        database=db_status,
        qdrant=qdrant_status,
    )


@app.post("/api/chat", response_model=ChatResponse)
async def chat(request: ChatRequest) -> ChatResponse:
    """Chat endpoint with RAG retrieval

    Args:
        request: Chat request with user query

    Returns:
        Chat response with answer and sources
    """
    if not request.query.strip():
        raise HTTPException(status_code=400, detail="Query cannot be empty")

    # For now, return a placeholder response
    # In production, this would use LangChain + Qdrant for actual RAG
    return ChatResponse(
        answer="This is a placeholder response. RAG integration coming soon.",
        sources=[
            {
                "title": "Module 1: Week 1",
                "url": "/docs/01-Module-1-ROS2/01-Week-1-Foundations",
                "excerpt": "Introduction to ROS 2 and robotics fundamentals...",
            }
        ],
    )


@app.get("/api/users/me")
async def get_current_user():
    """Get current user profile (placeholder for Better-Auth)"""
    return {
        "id": 1,
        "email": "user@example.com",
        "hardware_profile": "rtx",
        "full_name": "Test User",
    }


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=8000)
