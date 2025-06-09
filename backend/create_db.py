# --- db.py ------------------------------------------------------
from sqlalchemy import create_engine, Column, Integer, Float, DateTime
from sqlalchemy.orm import declarative_base, sessionmaker, scoped_session
import datetime, os

DATABASE_URL = os.getenv("DATABASE_URL", "sqlite:///data.db")

engine = create_engine(
    DATABASE_URL,
    # let SQLite be used from several threads
    connect_args={"check_same_thread": False} if DATABASE_URL.startswith("sqlite") else {},
    echo=False
)
Session = scoped_session(sessionmaker(bind=engine))

Base = declarative_base()

class Battery(Base):
    __tablename__ = "battery"
    id          = Column(Integer, primary_key=True)      # we’ll keep exactly one row (id = 1)
    level       = Column(Float, nullable=False)
    updated_at  = Column(DateTime, default=datetime.datetime.utcnow)

Base.metadata.create_all(engine)


# ---------- helper API ----------
def get_battery(db=None) -> float | None:
    """Return last stored battery level or None."""
    owns_session = db is None
    db = db or Session()
    row = db.get(Battery, 1)
    if owns_session: db.close()
    return row.level if row else None

def set_battery(level: float, db=None) -> None:
    owns_session = db is None
    db = db or Session()
    row = db.get(Battery, 1)
    now = datetime.datetime.utcnow()
    if row:
        row.level, row.updated_at = level, now
    else:
        db.add(Battery(id=1, level=level, updated_at=now))
    db.commit()
    if owns_session: db.close()
