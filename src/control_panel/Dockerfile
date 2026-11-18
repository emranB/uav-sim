FROM python:3.11-slim
WORKDIR /workspace
COPY pyproject.toml ./
COPY src ./src
RUN pip install --no-cache-dir . && useradd --create-home app && mkdir outputs && chown app outputs
USER app
EXPOSE 8080
CMD ["python", "-m", "common.runtime"]
