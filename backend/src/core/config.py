"""Application configuration"""

from pydantic_settings import BaseSettings


class Settings(BaseSettings):
    """Application settings loaded from environment variables"""

    app_name: str = "Physical AI Textbook Backend"
    debug: bool = False
    database_url: str = "postgresql://postgres:postgres@localhost:5432/physical_ai"
    qdrant_url: str = "http://localhost:6333"
    qdrant_collection: str = "textbook_chunks"

    class Config:
        env_file = ".env"
        case_sensitive = False


settings = Settings()
