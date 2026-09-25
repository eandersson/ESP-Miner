import { ComponentFixture, TestBed } from '@angular/core/testing';

import { NetworkEditComponent } from './network.edit.component';
import { provideHttpClient, withXhr } from '@angular/common/http';
import { provideToastr } from 'ngx-toastr';
import { DialogService } from 'src/app/services/dialog.service';

describe('NetworkEditComponent', () => {
  let component: NetworkEditComponent;
  let fixture: ComponentFixture<NetworkEditComponent>;

  beforeEach(() => {
    TestBed.configureTestingModule({
      declarations: [NetworkEditComponent],
      providers: [provideHttpClient(withXhr()), provideToastr(), DialogService]
    });
    fixture = TestBed.createComponent(NetworkEditComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
