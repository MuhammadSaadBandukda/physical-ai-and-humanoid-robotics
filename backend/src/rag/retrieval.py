"""RAG Retrieval Service: Query Qdrant for relevant chunks"""

from typing import List, Dict
from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct
import numpy as np


class RAGRetrievalService:
    """Service to retrieve relevant chunks from Qdrant"""

    def __init__(self, qdrant_url: str, collection_name: str = "textbook_chunks"):
        """Initialize the retrieval service

        Args:
            qdrant_url: URL to Qdrant vector database
            collection_name: Name of the Qdrant collection
        """
        self.client = QdrantClient(url=qdrant_url)
        self.collection_name = collection_name

    def ensure_collection_exists(self) -> bool:
        """Ensure the collection exists in Qdrant

        Returns:
            True if collection exists or was created
        """
        try:
            self.client.get_collection(self.collection_name)
            return True
        except Exception:
            # Collection doesn't exist; will be created during ingestion
            return False

    def retrieve(
        self, query_embedding: List[float], limit: int = 5
    ) -> List[Dict]:
        """Retrieve relevant chunks from Qdrant

        Args:
            query_embedding: Vector embedding of the query
            limit: Maximum number of results to return

        Returns:
            List of relevant chunks with metadata
        """
        try:
            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=limit,
            )

            chunks = []
            for result in results:
                chunk = {
                    "score": result.score,
                    "text": result.payload.get("text", ""),
                    "source": result.payload.get("relative_path", ""),
                    "title": result.payload.get("title", ""),
                    "module": result.payload.get("module", ""),
                }
                chunks.append(chunk)

            return chunks
        except Exception as e:
            print(f"Error retrieving chunks: {e}")
            return []
