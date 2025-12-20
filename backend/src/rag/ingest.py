"""RAG Ingestion Service: Extract and vectorize MDX content"""

import os
import re
from pathlib import Path
from typing import List, Dict, Optional


class RAGIngestionService:
    """Service to ingest and process MDX files for RAG"""

    def __init__(self, docs_path: str = "frontend/docs"):
        """Initialize the ingestion service

        Args:
            docs_path: Path to the MDX documentation files
        """
        self.docs_path = Path(docs_path)
        self.chunks: List[Dict] = []

    def extract_mdx_files(self) -> List[Path]:
        """Extract all MDX files from docs directory

        Returns:
            List of Path objects pointing to MDX files
        """
        return list(self.docs_path.rglob("*.md*"))

    def parse_mdx_file(self, file_path: Path) -> Dict:
        """Parse a single MDX file and extract metadata and content

        Args:
            file_path: Path to the MDX file

        Returns:
            Dictionary with metadata and content
        """
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                text = f.read()

            # Simple frontmatter parsing (YAML between --- markers)
            metadata = {}
            content = text
            if text.startswith("---"):
                parts = text.split("---", 2)
                if len(parts) >= 3:
                    content = parts[2].strip()
                    # Simple metadata extraction
                    for line in parts[1].strip().split("\n"):
                        if ":" in line:
                            key, value = line.split(":", 1)
                            metadata[key.strip()] = value.strip()
            else:
                content = text

            # Extract module and week from directory structure
            relative_path = file_path.relative_to(self.docs_path)
            path_parts = relative_path.parts

            module = None
            week = None
            if len(path_parts) > 0:
                module = path_parts[0]
            if len(path_parts) > 1:
                week = path_parts[1]

            return {
                "file_path": str(file_path),
                "relative_path": str(relative_path),
                "title": metadata.get("title", file_path.stem),
                "content": content,
                "module": module,
                "week": week,
                "metadata": metadata,
            }
        except Exception as e:
            print(f"Error parsing {file_path}: {e}")
            return None

    def chunk_content(self, text: str, chunk_size: int = 500) -> List[str]:
        """Split content into chunks for embedding

        Args:
            text: Content to chunk
            chunk_size: Approximate size of each chunk in characters

        Returns:
            List of text chunks
        """
        sentences = re.split(r"(?<=[.!?])\s+", text)
        chunks = []
        current_chunk = ""

        for sentence in sentences:
            if len(current_chunk) + len(sentence) < chunk_size:
                current_chunk += sentence + " "
            else:
                if current_chunk.strip():
                    chunks.append(current_chunk.strip())
                current_chunk = sentence + " "

        if current_chunk.strip():
            chunks.append(current_chunk.strip())

        return chunks

    def ingest_all(self) -> List[Dict]:
        """Ingest all MDX files and create chunks

        Returns:
            List of chunks ready for embedding
        """
        mdx_files = self.extract_mdx_files()
        all_chunks = []

        for file_path in mdx_files:
            doc = self.parse_mdx_file(file_path)
            if not doc:
                continue

            chunks = self.chunk_content(doc["content"])
            for chunk_text in chunks:
                chunk_entry = {
                    "text": chunk_text,
                    "file_path": doc["file_path"],
                    "relative_path": doc["relative_path"],
                    "title": doc["title"],
                    "module": doc["module"],
                    "week": doc["week"],
                    "metadata": doc["metadata"],
                }
                all_chunks.append(chunk_entry)

        self.chunks = all_chunks
        return all_chunks
