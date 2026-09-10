import pytest
from tools.export_ik_candidate_gallery import aligned_relations


def row(batch,phase,grasp='g',place=None,prefetch=False):
    return dict(batch=batch,phase=phase,grasp_label=grasp,place_label=place,prefetch=prefetch)


def test_aligned_graphs_do_not_borrow_grasp_from_other_relation_or_prefetch():
    pre=row(1,'pregrasp');grasp=row(2,'grasp');h=row(3,'hover',place='p');r=row(3,'release',place='p')
    rows=[row(0,'pregrasp','other',prefetch=True),pre,grasp,h,r,
          row(4,'hover','other','p'),row(4,'release','other','p')]
    aligned=aligned_relations(rows)
    assert aligned[0]==(pre,grasp,h,r)
    assert aligned[1][:2]==(None,None)


def test_aligned_graphs_preserve_retry_context():
    first=[row(1,'pregrasp'),row(2,'grasp'),row(3,'hover',place='p'),row(3,'release',place='p')]
    second=[row(4,'pregrasp'),row(5,'grasp'),row(6,'hover',place='p'),row(6,'release',place='p')]
    result=aligned_relations(first+second)
    assert [x[0]['batch'] for x in result]==[1,4]


def test_mismatched_release_identity_is_rejected():
    with pytest.raises(ValueError):aligned_relations([row(1,'hover','g','p'),row(1,'release','other','p')])
